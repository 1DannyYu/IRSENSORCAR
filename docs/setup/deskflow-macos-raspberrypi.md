# Deskflow Setup Guide: macOS (Apple Silicon) + Raspberry Pi (Wayland/labwc)

> **Scenario:** One macOS system (M-series Apple Silicon) acting as the **Server** (sharing its keyboard and mouse),
> and one Raspberry Pi 5 (Raspberry Pi OS with **labwc / wlroots** desktop environment in a Wayland session) acting as the **Client**.
>
> This guide is documented based on a real setup process, recording common pitfalls and their working solutions.

---

## Environment Information

| Item                                 | Details                                                                                                             |
| ------------------------------------ | ------------------------------------------------------------------------------------------------------------------- |
| **Server**                     | macOS, M-series Apple Silicon                                                                                       |
| **Client**                     | Raspberry Pi 5, Raspberry Pi OS                                                                                     |
| **Client Desktop Environment** | `labwc:wlroots` (Verify with `echo $XDG_CURRENT_DESKTOP`)                                                       |
| **Client Session Type**        | `wayland` (Verify with `echo $XDG_SESSION_TYPE`)                                                                |
| **Software**                   | [Deskflow](https://github.com/deskflow/deskflow) v1.26.0                                                             |
| **Client Installation Method** | Flatpak (Required because default Qt/libei versions on Raspberry Pi OS are usually too old for native Deskflow GUI) |

---

## Part 1: macOS (Server Side) Setup

### 1.1 Install via Homebrew

```bash
brew tap deskflow/tap
brew install deskflow
```

**Troubleshooting Pitfall:** If you encounter the following error:

```
Error: Refusing to load cask deskflow/tap/deskflow from untrusted tap deskflow/tap.
```

This indicates your Homebrew environment is a customized or managed version (standard Homebrew does not enforce this restriction). Solution:

```bash
brew trust deskflow/tap
brew install deskflow
```

### 1.2 Resolve "Damaged App" Warning

Deskflow is not notarized by Apple certificates, so macOS may show a "damaged app" error on first launch. Run:

```bash
xattr -c /Applications/Deskflow.app
```

### 1.3 Grant System Permissions

Open `Deskflow.app` and navigate to **System Settings → Privacy & Security**:

- Enable **Accessibility** permissions (for both Deskflow app and the `deskflow` binary process).
- On **macOS Sequoia or later**: Enable Deskflow under **Local Network** settings.

### 1.4 Configure as Server

- Launch Deskflow → Select **"Use this computer's keyboard and mouse (make this computer the server)"**.
- The app header will display a `Suggested IP`. Note down this IP address (e.g., `192.168.1.246`), as it will be needed on the Client side.
- If you want to confirm the Mac's Wi-Fi IP address from Terminal, you can also run:

```bash
ipconfig getifaddr en0
```

- Click **Configure Server → Computers** tab, add the screen name of the Client (Raspberry Pi), and drag it to match its physical position (Left / Right / Above / Below).

---

## Part 2: Raspberry Pi (Client Side) Setup

### 2.1 Connect via SSH

On the Raspberry Pi desktop, the default shortcut to open **Terminal** is:

```bash
Ctrl + Alt + T
```

First, identify the username and IP address on the local Raspberry Pi terminal (or through your router):

```bash
whoami          # Confirm username
hostname -I     # Note uppercase -I to obtain actual network IP (do not use lowercase -i which returns 127.0.1.1)
```

Connect from your Mac:

```bash
ssh <username>@<raspberry_pi_ip>
# Or try the mDNS hostname (if supported on your local network):
ssh <username>@raspberrypi.local
```

Example commands from this documented setup:

```bash
ssh dannypi@192.168.1.27
ssh jasonmacbbookpro@192.168.1.246
```

### 2.2 Install Flatpak and Deskflow

```bash
sudo apt update
sudo apt install flatpak
flatpak remote-add --if-not-exists flathub https://dl.flathub.org/repo/flathub.flatpakrepo
flatpak search deskflow      # Verify org.deskflow.deskflow is available
flatpak install --user flathub org.deskflow.deskflow
```

**Troubleshooting Pitfall 1:** Adding the Flathub remote using the legacy URL (`flathub.org/repo/...`) may fail silently, causing subsequent `flatpak search` and `flatpak install` commands to fail finding packages. Use `flatpak remotes` to verify if the remote exists; if missing, re-add using `https://dl.flathub.org/repo/flathub.flatpakrepo`.

**Troubleshooting Pitfall 2:** Installing system-wide with default `flatpak install flathub ...` will prompt for password verification (polkit) for each dependency. Adding the `--user` flag installs packages in the user directory and bypasses repeated password prompts.

### 2.3 Launch Deskflow (Must be executed on the physical Raspberry Pi desktop, not over SSH)

Open a terminal on the display connected to the Raspberry Pi:

```bash
flatpak run org.deskflow.deskflow
```

> ⚠️ Deskflow client opens a GUI window. A text-only SSH terminal cannot render it; a physical monitor connection or VNC session is required.

### 2.4 Configure as Client

- Select **"Use another computer's mouse and keyboard (make this computer the client)"**.
- In the "Connect to" field, enter the Mac's IP address (recorded in Part 1.4).
- Click **Connect**.

---

## Part 3: Troubleshooting Connection Failures (Real-world Pitfall)

### Symptoms

The Client continuously displays:

```
Failed to connect to the server 'x.x.x.x'.
Please check your TLS and firewall settings.
```

The Server logs repeatedly show:

```
WARNING: client connection may not be secure
ERROR: failed to accept secure socket
```

While this appears to be a TLS or firewall issue, **it is actually a misleading error**. The real root cause is located earlier in the Client log:

```
WARNING: can't open xkb display during reading languages
ERROR: failed to initialize remote desktop session: GDBus
```

### Root Cause

The Wayland compositor on Raspberry Pi OS is `labwc` (a `wlroots`-based compositor). On Linux/Wayland, Deskflow requests screen capture and remote input interface capabilities (`ScreenCast` / `RemoteDesktop`) via `xdg-desktop-portal`. These interfaces are **only implemented by `xdg-desktop-portal-wlr`**, and are not supported by `xdg-desktop-portal-gtk`.

Even if both portal packages are installed, the main `xdg-desktop-portal` daemon lacks routing configuration specifying that `ScreenCast` and `RemoteDesktop` requests must be forwarded to `wlr` instead of the default `gtk`. Consequently, the portal request fails immediately, causing the Client to crash before completing the TLS handshake. On the Server side, the aborted connection is flagged as an "incomplete insecure connection", producing the misleading TLS/firewall error.

### Solution

Run the following commands on the Raspberry Pi:

```bash
mkdir -p ~/.config/xdg-desktop-portal
cat > ~/.config/xdg-desktop-portal/labwc-portals.conf << 'EOF'
[preferred]
default=gtk
org.freedesktop.impl.portal.ScreenCast=wlr
org.freedesktop.impl.portal.RemoteDesktop=wlr
EOF

systemctl --user restart xdg-desktop-portal xdg-desktop-portal-wlr
```

Then restart the Deskflow client:

```bash
flatpak run org.deskflow.deskflow
```

Click **Connect** again. The logs will no longer throw `GDBus` / `failed to initialize remote desktop session` errors, and the connection will establish successfully.

---

## Part 4: Optional Raspberry Pi Power Health Check

If you are powering the Raspberry Pi from a battery pack, DC module, or other non-standard power source, it is useful to confirm that the input voltage is still within a healthy range and that the system is not being throttled.

Run the following commands on the Raspberry Pi:

```bash
vcgencmd pmic_read_adc EXT5V_V
vcgencmd get_throttled
```

Example output from this documented setup:

```bash
dannypi@Danny-Raspberrypi5-8gram-225gssd:~ $ vcgencmd pmic_read_adc EXT5V_V
EXT5V_V volt(24)=4.85616000V

dannypi@Danny-Raspberrypi5-8gram-225gssd:~ $ vcgencmd get_throttled
throttled=0x0
```

### Interpretation

| Check | Result | Status | Notes |
|---|---|---|---|
| `EXT5V_V` | `4.856V` | Acceptable | Slightly low, but still within a normal operating range |
| `get_throttled` | `0x0` | Normal | No undervoltage, throttling, or overheating flags are present |

### What This Means

- `4.856V` is above the commonly cited `4.8V` comfort threshold, so the system is currently operating normally.
- Raspberry Pi undervoltage warnings are typically triggered when the supply drops to around `4.63V` or below.
- `throttled=0x0` means:
  - no undervoltage is currently detected
  - no CPU frequency throttling is active
  - no overheating condition is active
  - no performance limiting flags are set

### Battery-Powered Setup Note

If the Raspberry Pi is powered by a lithium battery, the input voltage may gradually decrease as the battery discharges. Even if the current reading is healthy, it is a good idea to repeat this check periodically during longer sessions.

---

## Diagnostic Commands Cheat Sheet

| Purpose                                  | Command                                                          |
| ---------------------------------------- | ---------------------------------------------------------------- |
| Check Wayland/Desktop environment        | `echo $XDG_SESSION_TYPE` / `echo $XDG_CURRENT_DESKTOP`       |
| Get the Mac server IP on Wi-Fi (`en0`) | `ipconfig getifaddr en0`                                       |
| Get physical IP of Raspberry Pi          | `hostname -I` (uppercase `-I`)                               |
| Open Terminal on Raspberry Pi desktop    | `Ctrl + Alt + T`                                               |
| Check Raspberry Pi 5V input voltage      | `vcgencmd pmic_read_adc EXT5V_V`                              |
| Check throttling / undervoltage flags    | `vcgencmd get_throttled`                                      |
| Verify Flatpak remotes                   | `flatpak remotes`                                              |
| Check supported Deskflow architecture    | `flatpak search deskflow`                                      |
| Verify installed portal packages         | `dpkg -l \| grep xdg-desktop-portal`                            |
| Check status of`wlr` portal service    | `systemctl --user status xdg-desktop-portal-wlr`               |
| View`wlr` portal logs                  | `journalctl --user -u xdg-desktop-portal-wlr -n 50 --no-pager` |
| Check Flatpak app permissions            | `flatpak info --show-permissions org.deskflow.deskflow`        |

---

## Quick Summary (TL;DR)

1. **macOS**: Install via Homebrew → `brew trust` to fix untrusted tap error → `xattr -c` to fix damaged app warning → Grant Accessibility permissions → Set as Server.
2. **Raspberry Pi**: Use **Flatpak (`--user` flag)** instead of native package → Add Flathub remote with `dl.flathub.org` URL → Run on physical display (GUI app) → Set as Client.
3. **If TLS/firewall error occurs**: **Do not troubleshoot TLS first**. Check Client logs for `GDBus` or `remote desktop session` errors — this indicates Wayland portal routing failure to `xdg-desktop-portal-wlr`. Follow Part 3 to create the configuration file and restart portal services.
4. **If using battery power**: Run `vcgencmd pmic_read_adc EXT5V_V` and `vcgencmd get_throttled` to confirm voltage and throttling status.
