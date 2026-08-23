# Reaching the Raspberry Pi From a Mac

This project is developed on a Mac but runs on a Raspberry Pi 5. Everything in
[`src/carbot/`](../../src/carbot/) and [`examples/`](../../examples/) must execute on the Pi,
because only the Pi is wired to the NeZha driver board over I2C. This guide covers the ways to
reach that Pi, when to use each one, and what not to do over a remote session.

---

## Pick a Method

| Method | Use it for | Works outside your home network? | Cost |
|---|---|---|---|
| **SSH** | Terminal work: `git`, `uv`, running scripts, editing config. The default choice. | Only with extra setup (VPN or tunnel) | Free |
| **Raspberry Pi Connect** | Reaching the Pi from anywhere — browser-based desktop **and** shell. | **Yes** — this is its whole point | Free for personal use |
| **VNC** | Full desktop GUI over the local network. | No | Free |
| **Deskflow** | One keyboard and mouse shared across a Mac and a Pi sitting on the same desk. Not a remote-access tool. | No | Free |

Most days you only need SSH. Reach for Raspberry Pi Connect when you are away from home, and for
VNC or Deskflow only when you genuinely need the graphical desktop.

---

## 1. SSH — The Baseline

macOS ships with an `ssh` client, so nothing needs installing on the Mac side.

### 1.1 Enable SSH on the Pi

Pick whichever is convenient:

```bash
sudo raspi-config
```

Then `Interface Options` → `SSH` → `Yes`.

Or enable it while flashing the SD card, in Raspberry Pi Imager's **Customisation → Remote
Access**; or on an already-flashed card, create an empty file:

```bash
sudo touch /boot/firmware/ssh && sudo reboot
```

### 1.2 Find the Pi

Run these on the Pi itself (open a terminal on its desktop with `Ctrl + Alt + T`):

```bash
whoami
hostname -I
```

`hostname -I` must use an **uppercase** `-I`. The lowercase `-i` returns `127.0.1.1`, which is
useless for connecting.

### 1.3 Connect

```bash
ssh <username>@<raspberry-pi-ip>
```

If your network supports mDNS, the hostname works too and survives DHCP changing the IP:

```bash
ssh <username>@raspberrypi.local
```

### 1.4 Stop Typing the Password

Generate a key on the Mac, then copy the public half to the Pi:

```bash
ssh-keygen -t ed25519
ssh-copy-id <username>@<raspberry-pi-ip>
```

Then add an alias to `~/.ssh/config` on the Mac so `ssh carpi` is all you need:

```
Host carpi
    HostName raspberrypi.local
    User <username>
```

### 1.5 Move Files

Prefer `git` for source code — the Pi clones this repository directly, per
[raspberry-pi-first-run.md](raspberry-pi-first-run.md). Use `scp` or `rsync` for one-off files such
as logs or photos that are not committed.

```bash
scp notes.txt <username>@<raspberry-pi-ip>:
scp <username>@<raspberry-pi-ip>:capture.jpg .
rsync -avz -e ssh <username>@<raspberry-pi-ip>:~/logs/ ./logs/
```

### 1.6 Survive a Dropped Connection

An SSH session dies with your Wi-Fi, and anything running in it dies too. For a long test, run it
inside `tmux` so the process keeps going and you can reattach later.

```bash
sudo apt install -y tmux
tmux new -s carbot
```

Detach with `Ctrl + b` then `d`. Reattach later with:

```bash
tmux attach -t carbot
```

Note that this convenience does **not** apply to motor or servo scripts — see the Safety section
below.

---

## 2. Raspberry Pi Connect — Reaching Home From Anywhere

[Raspberry Pi Connect](https://www.raspberrypi.com/documentation/services/connect.html) is
Raspberry Pi's own remote-access service. You sign in at
[connect.raspberrypi.com](https://connect.raspberrypi.com) and get to the Pi through a browser —
no port forwarding, no firewall changes, no chasing a changing home IP address. **Personal
accounts are free.**

It offers two modes:

| Mode | What you get | Requirements |
|---|---|---|
| **Screen sharing** | The full Pi desktop in a browser tab | Raspberry Pi OS Bookworm or later, **Wayland** session, and a desktop actually logged in |
| **Remote shell** | A terminal in a browser tab | Any variant, including Lite |

This project's Pi runs Raspberry Pi OS with a `labwc` Wayland session (the same environment
documented in [deskflow-macos-raspberrypi.md](deskflow-macos-raspberrypi.md)), so **screen sharing
is supported here**.

### Enable it

Connect ships preinstalled on Raspberry Pi OS Desktop and Full. On the Pi:

```bash
rpi-connect on
rpi-connect signin
```

`signin` prints a verification URL. Open it, sign in with a Raspberry Pi ID, and the device is
linked. Check state at any time with:

```bash
rpi-connect status
```

### How the traffic flows

Connect tries to establish a direct connection between your browser and the Pi. When the network
does not allow that, traffic falls back to Raspberry Pi's relay servers, which keep only
operational metadata. Relayed sessions are slower than direct ones — if screen sharing feels
sluggish from a café, that is usually why.

---

## 3. VNC — Local Desktop

When you want the graphical desktop and both machines are on the same network, VNC is lighter than
Connect. Enable it through `sudo raspi-config` → `Interface Options` → `VNC`, or the desktop's
Control Centre → Interfaces. Raspberry Pi's documentation recommends
[TigerVNC](https://tigervnc.org/) as the client.

---

## 4. Deskflow — One Keyboard, Two Machines

Deskflow shares the Mac's keyboard and mouse with a Pi sitting on the same desk, so the cursor
slides between two physical screens. It is a comfort tool for a two-machine desk, **not** a way to
reach the Pi from elsewhere.

Setting it up on macOS + Raspberry Pi OS Wayland has real pitfalls — in particular a misleading
TLS error that is actually an `xdg-desktop-portal` routing problem. The full walkthrough, including
that fix, is in [deskflow-macos-raspberrypi.md](deskflow-macos-raspberrypi.md).

---

## Safety

**Do not run motor or servo scripts over a remote session unless someone is physically beside the
robot.**

This is not a general caution, it is how the scripts are built.
[`examples/04_servo_check.py`](../../examples/04_servo_check.py) pauses before every movement and
tells the operator to cut main power if anything looks wrong. That instruction assumes a person
within arm's reach of the power switch. A browser tab in another city cannot do that.

| Safe over a remote session | Requires someone at the robot |
|---|---|
| `git pull`, `uv sync`, editing config | `examples/02_motor_check.py` |
| `uv run pytest` | `examples/03_motor_drive.py` |
| `examples/01_i2c_probe.py` (communication only, flashes an LED) | `examples/04_servo_check.py` |
| `i2cdetect -y 1`, `vcgencmd get_throttled` | Anything after Step 5 of [raspberry-pi-first-run.md](raspberry-pi-first-run.md) |

Related: the wiring and power rules in [CLAUDE.md](../../CLAUDE.md) and the safety notes in
[README.md](../../README.md) still apply regardless of how you connected.

---

## When You Get Stuck

### Show the screen to Claude

Much of this setup happens in GUIs — `raspi-config` menus, macOS **System Settings → Privacy &
Security**, the Deskflow window, the Connect dashboard. Describing a stuck menu in words is slow
and error-prone. Take a screenshot and paste it into [claude.ai](https://claude.ai) instead: it
reads the menu state, the highlighted option, and the error text directly, which is usually faster
than a round of "which checkbox exactly?".

Screenshot shortcuts:

| Platform | Shortcut |
|---|---|
| macOS | `Cmd + Shift + 4` (region) / `Cmd + Shift + 3` (full screen) |
| Raspberry Pi OS | `Print Screen`, or `scrot` / `grim` from a terminal |

For terminal errors, pasting the text is better than a screenshot — it is searchable and quotable.

### Use the official Raspberry Pi resources

Raspberry Pi's own documentation is unusually thorough and is the right first stop for anything
about the Pi itself rather than this project. It is free and it covers far more than most people
realise — OS configuration, `config.txt`, kernel building, remote access, cameras, and the hardware
specifications for every board.

| Resource | Link |
|---|---|
| Documentation home | <https://www.raspberrypi.com/documentation/> |
| Remote access | <https://www.raspberrypi.com/documentation/computers/remote-access.html> |
| Raspberry Pi Connect | <https://www.raspberrypi.com/documentation/services/connect.html> |
| Community forums | <https://forums.raspberrypi.com/> |

The documentation site also has an **"Ask a question"** box at the top of the page that answers
from the official documentation, at no cost. It is worth trying before searching the forums,
because it cites the official material rather than a stranger's five-year-old thread.

---

## Cheat Sheet

| Purpose | Command |
|---|---|
| Find the Pi's IP | `hostname -I` (uppercase `-I`) |
| Find the Mac's Wi-Fi IP | `ipconfig getifaddr en0` |
| Open a terminal on the Pi desktop | `Ctrl + Alt + T` |
| Connect over SSH | `ssh <username>@raspberrypi.local` |
| Set up key login | `ssh-keygen -t ed25519` then `ssh-copy-id <username>@<ip>` |
| Copy a file to the Pi | `scp <file> <username>@<ip>:` |
| Start a detachable session | `tmux new -s carbot` |
| Turn on Pi Connect | `rpi-connect on` then `rpi-connect signin` |
| Check Pi Connect state | `rpi-connect status` |
| Enable SSH / VNC / I2C | `sudo raspi-config` → `Interface Options` |
| Confirm the NeZha board responds | `i2cdetect -y 1` (expects `40`) |
