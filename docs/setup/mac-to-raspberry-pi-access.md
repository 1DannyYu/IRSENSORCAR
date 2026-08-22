# Reaching the Raspberry Pi From a Mac — 從 Mac 連上樹莓派

This project is developed on a Mac but runs on a Raspberry Pi 5. Everything in
[`src/carbot/`](../../src/carbot/) and [`examples/`](../../examples/) must execute on the Pi,
because only the Pi is wired to the NeZha driver board over I2C. This guide covers the ways to
reach that Pi, when to use each one, and what not to do over a remote session.

本專案在 Mac 上開發，但實際跑在 Raspberry Pi 5 上。[`src/carbot/`](../../src/carbot/) 和
[`examples/`](../../examples/) 底下的東西一定要在 Pi 上執行，因為只有 Pi 透過 I2C 接著 NeZha
驅動板。這份文件說明有哪些連線方式、什麼情境用哪一種，以及遠端操作時絕對不能做什麼。

---

## Pick a Method — 選擇連線方式

| Method 方式 | Use it for 適用情境 | Works outside your home network? 出門在外可用？ | Cost 費用 |
|---|---|---|---|
| **SSH** | Terminal work: `git`, `uv`, running scripts, editing config. The default choice.<br>終端機作業：`git`、`uv`、跑腳本、改設定。預設選項。 | Only with extra setup (VPN or tunnel)<br>需額外設定（VPN 或 tunnel） | Free 免費 |
| **Raspberry Pi Connect** | Reaching the Pi from anywhere — browser-based desktop **and** shell.<br>人在外面連回家中的 Pi，瀏覽器裡直接用桌面**或**終端機。 | **Yes** — this is its whole point<br>**可以** — 它就是為此而生 | Free for personal use<br>個人帳號免費 |
| **VNC** | Full desktop GUI over the local network.<br>在區網內操作完整桌面 GUI。 | No 不行 | Free 免費 |
| **Deskflow** | One keyboard and mouse shared across a Mac and a Pi sitting on the same desk. Not a remote-access tool.<br>Mac 和 Pi 擺在同一張桌上時共用一組鍵鼠。**不是**遠端連線工具。 | No 不行 | Free 免費 |

Most days you only need SSH. Reach for Raspberry Pi Connect when you are away from home, and for
VNC or Deskflow only when you genuinely need the graphical desktop.

多數時候只需要 SSH。人不在家時用 Raspberry Pi Connect；真的需要圖形桌面時才用 VNC 或 Deskflow。

---

## 1. SSH — The Baseline 基本盤

macOS ships with an `ssh` client, so nothing needs installing on the Mac side.

macOS 內建 `ssh`，Mac 這端不用裝任何東西。

### 1.1 Enable SSH on the Pi 在 Pi 上開啟 SSH

Pick whichever is convenient — 三種擇一即可：

```bash
sudo raspi-config
```

Then `Interface Options` → `SSH` → `Yes`.

或在燒錄 SD 卡時於 Raspberry Pi Imager 的 **Customisation → Remote Access** 直接勾選；
或對著已燒好的卡建立一個空檔案：

```bash
sudo touch /boot/firmware/ssh && sudo reboot
```

### 1.2 Find the Pi 找到 Pi

Run these on the Pi itself (open a terminal on its desktop with `Ctrl + Alt + T`):

在 Pi 上執行（桌面按 `Ctrl + Alt + T` 開終端機）：

```bash
whoami
hostname -I
```

`hostname -I` must use an **uppercase** `-I`. The lowercase `-i` returns `127.0.1.1`, which is
useless for connecting.

`hostname -I` 一定要用**大寫** `-I`。小寫 `-i` 回傳 `127.0.1.1`，拿來連線是沒用的。

### 1.3 Connect 連線

```bash
ssh <username>@<raspberry-pi-ip>
```

If your network supports mDNS, the hostname works too and survives DHCP changing the IP:

如果你的網路支援 mDNS，可以直接用主機名，IP 被 DHCP 換掉也不受影響：

```bash
ssh <username>@raspberrypi.local
```

### 1.4 Stop Typing the Password 免密碼登入

Generate a key on the Mac, then copy the public half to the Pi:

在 Mac 上產生金鑰，再把公鑰送到 Pi：

```bash
ssh-keygen -t ed25519
ssh-copy-id <username>@<raspberry-pi-ip>
```

Then add an alias to `~/.ssh/config` on the Mac so `ssh carpi` is all you need:

接著在 Mac 的 `~/.ssh/config` 加一個別名，之後只要打 `ssh carpi`：

```
Host carpi
    HostName raspberrypi.local
    User <username>
```

### 1.5 Move Files 傳檔案

Prefer `git` for source code — the Pi clones this repository directly, per
[raspberry-pi-first-run.md](raspberry-pi-first-run.md). Use `scp` or `rsync` for one-off files such
as logs or photos that are not committed.

原始碼優先走 `git` — Pi 直接 clone 這個 repo，見
[raspberry-pi-first-run.md](raspberry-pi-first-run.md)。沒有進版控的零星檔案（log、照片）才用
`scp` 或 `rsync`。

```bash
scp notes.txt <username>@<raspberry-pi-ip>:
scp <username>@<raspberry-pi-ip>:capture.jpg .
rsync -avz -e ssh <username>@<raspberry-pi-ip>:~/logs/ ./logs/
```

### 1.6 Survive a Dropped Connection 斷線不中斷

An SSH session dies with your Wi-Fi, and anything running in it dies too. For a long test, run it
inside `tmux` so the process keeps going and you can reattach later.

SSH session 會跟著 Wi-Fi 一起斷，裡面跑的東西也會一起死。長時間測試請放進 `tmux`，
process 會繼續跑，之後可以再接回去。

```bash
sudo apt install -y tmux
tmux new -s carbot
```

Detach with `Ctrl + b` then `d`. Reattach later with:

按 `Ctrl + b` 再按 `d` 離開。之後用這個接回來：

```bash
tmux attach -t carbot
```

Note that this convenience does **not** apply to motor or servo scripts — see the Safety section
below.

注意這個便利性**不適用**於馬達或舵機腳本 — 見下方的安全守則。

---

## 2. Raspberry Pi Connect — Reaching Home From Anywhere 人在外面連回家

[Raspberry Pi Connect](https://www.raspberrypi.com/documentation/services/connect.html) is
Raspberry Pi's own remote-access service. You sign in at
[connect.raspberrypi.com](https://connect.raspberrypi.com) and get to the Pi through a browser —
no port forwarding, no firewall changes, no chasing a changing home IP address. **Personal
accounts are free.**

[Raspberry Pi Connect](https://www.raspberrypi.com/documentation/services/connect.html)
是樹莓派官方自己的遠端連線服務。到 [connect.raspberrypi.com](https://connect.raspberrypi.com)
登入，就能從瀏覽器連到 Pi — 不用設 port forwarding、不用改防火牆、不用追家裡會變動的
對外 IP。**個人帳號免費。**

It offers two modes — 提供兩種模式：

| Mode 模式 | What you get 得到什麼 | Requirements 需求 |
|---|---|---|
| **Screen sharing** 螢幕分享 | The full Pi desktop in a browser tab<br>瀏覽器分頁裡的完整 Pi 桌面 | Raspberry Pi OS Bookworm or later, **Wayland** session, and a desktop actually logged in<br>Bookworm 以上、**Wayland** session、且桌面已登入 |
| **Remote shell** 遠端終端機 | A terminal in a browser tab<br>瀏覽器分頁裡的終端機 | Any variant, including Lite<br>任何版本，含 Lite |

This project's Pi runs Raspberry Pi OS with a `labwc` Wayland session (the same environment
documented in [deskflow-macos-raspberrypi.md](deskflow-macos-raspberrypi.md)), so **screen sharing
is supported here**.

本專案的 Pi 跑的是 Raspberry Pi OS + `labwc` Wayland session（與
[deskflow-macos-raspberrypi.md](deskflow-macos-raspberrypi.md) 記錄的環境相同），所以**這台可以用
螢幕分享**。

### Enable it 啟用

Connect ships preinstalled on Raspberry Pi OS Desktop and Full. On the Pi:

Raspberry Pi OS Desktop 與 Full 版已預裝。在 Pi 上執行：

```bash
rpi-connect on
rpi-connect signin
```

`signin` prints a verification URL. Open it, sign in with a Raspberry Pi ID, and the device is
linked. Check state at any time with:

`signin` 會印出一組驗證網址。打開、用 Raspberry Pi ID 登入，裝置就綁定完成。
隨時可以檢查狀態：

```bash
rpi-connect status
```

### How the traffic flows 連線怎麼走

Connect tries to establish a direct connection between your browser and the Pi. When the network
does not allow that, traffic falls back to Raspberry Pi's relay servers, which keep only
operational metadata. Relayed sessions are slower than direct ones — if screen sharing feels
sluggish from a café, that is usually why.

Connect 會先嘗試在瀏覽器和 Pi 之間建立直連。網路環境不允許時，才改走樹莓派官方的 relay
伺服器，relay 只保留運作用的 metadata。走 relay 會比直連慢 — 在咖啡廳覺得螢幕分享卡卡的，
通常就是這個原因。

---

## 3. VNC — Local Desktop 區網桌面

When you want the graphical desktop and both machines are on the same network, VNC is lighter than
Connect. Enable it through `sudo raspi-config` → `Interface Options` → `VNC`, or the desktop's
Control Centre → Interfaces. Raspberry Pi's documentation recommends
[TigerVNC](https://tigervnc.org/) as the client.

想要圖形桌面、而且兩台在同一個網路時，VNC 比 Connect 輕量。用
`sudo raspi-config` → `Interface Options` → `VNC` 開啟，或從桌面的 Control Centre → Interfaces
開啟。官方文件推薦的 client 是 [TigerVNC](https://tigervnc.org/)。

---

## 4. Deskflow — One Keyboard, Two Machines 一組鍵鼠兩台機器

Deskflow shares the Mac's keyboard and mouse with a Pi sitting on the same desk, so the cursor
slides between two physical screens. It is a comfort tool for a two-machine desk, **not** a way to
reach the Pi from elsewhere.

Deskflow 讓 Mac 的鍵盤滑鼠分享給同一張桌上的 Pi，游標可以在兩台實體螢幕之間滑來滑去。
這是雙機桌面的順手工具，**不是**遠端連線方案。

Setting it up on macOS + Raspberry Pi OS Wayland has real pitfalls — in particular a misleading
TLS error that is actually an `xdg-desktop-portal` routing problem. The full walkthrough, including
that fix, is in [deskflow-macos-raspberrypi.md](deskflow-macos-raspberrypi.md).

在 macOS + Raspberry Pi OS Wayland 上設定它有幾個真的會踩到的坑 — 尤其是一個看起來像 TLS
問題、實際上是 `xdg-desktop-portal` 路由設定的錯誤。完整流程與解法在
[deskflow-macos-raspberrypi.md](deskflow-macos-raspberrypi.md)。

---

## Safety — 遠端操作安全守則

**Do not run motor or servo scripts over a remote session unless someone is physically beside the
robot.**

**除非有人實際站在機器旁邊，否則不要透過遠端 session 執行馬達或舵機腳本。**

This is not a general caution, it is how the scripts are built.
[`examples/other/04_servo_check.py`](../../examples/other/04_servo_check.py) pauses before every movement and
tells the operator to cut main power if anything looks wrong. That instruction assumes a person
within arm's reach of the power switch. A browser tab in another city cannot do that.

這不是泛泛的提醒，而是腳本本身的設計前提。
[`examples/other/04_servo_check.py`](../../examples/other/04_servo_check.py) 在每一次移動前都會暫停，
並要求操作者「有異常請直接關閉主電源」。這句話預設有人伸手就搆得到電源開關 —
在另一個城市的瀏覽器分頁做不到這件事。

| Safe over a remote session 遠端可做 | Requires someone at the robot 需要有人在機器旁 |
|---|---|
| `git pull`, `uv sync`, editing config | `examples/other/02_motor_check.py` |
| `uv run pytest` | `examples/other/03_motor_drive.py` |
| `examples/other/01_i2c_probe.py` (communication only, flashes an LED)<br>只測通訊，僅閃一下 LED | `examples/other/04_servo_check.py` |
| `i2cdetect -y 1`, `vcgencmd get_throttled` | Anything after Step 5 of [raspberry-pi-first-run.md](raspberry-pi-first-run.md) |

Related: the wiring and power rules in [CLAUDE.md](../../CLAUDE.md) and the safety notes in
[README.md](../../README.md) still apply regardless of how you connected.

相關：[CLAUDE.md](../../CLAUDE.md) 的接線與供電規則、[README.md](../../README.md) 的安全注意事項，
不論你用哪種方式連線都一樣適用。

---

## When You Get Stuck — 卡住的時候

### Show the screen to Claude 把畫面拿給 Claude 看

Much of this setup happens in GUIs — `raspi-config` menus, macOS **System Settings → Privacy &
Security**, the Deskflow window, the Connect dashboard. Describing a stuck menu in words is slow
and error-prone. Take a screenshot and paste it into [claude.ai](https://claude.ai) instead: it
reads the menu state, the highlighted option, and the error text directly, which is usually faster
than a round of "which checkbox exactly?".

這套設定有很多步驟發生在 GUI 裡 — `raspi-config` 選單、macOS 的 **System Settings → Privacy &
Security**、Deskflow 視窗、Connect 後台。用文字描述卡在哪一頁又慢又容易講錯。直接截圖貼到
[claude.ai](https://claude.ai)：它讀得懂選單狀態、被選中的項目和錯誤訊息，通常比來回問
「到底是哪個勾選框」快得多。

Screenshot shortcuts — 截圖快捷鍵：

| Platform | Shortcut |
|---|---|
| macOS | `Cmd + Shift + 4` (region 選取範圍) / `Cmd + Shift + 3` (full screen 全螢幕) |
| Raspberry Pi OS | `Print Screen`, or `scrot` / `grim` from a terminal |

For terminal errors, pasting the text is better than a screenshot — it is searchable and quotable.

終端機錯誤訊息請直接貼文字而不是截圖 — 文字可以搜尋、可以引用。

### Use the official Raspberry Pi resources 善用樹莓派官方資源

Raspberry Pi's own documentation is unusually thorough and is the right first stop for anything
about the Pi itself rather than this project. It is free and it covers far more than most people
realise — OS configuration, `config.txt`, kernel building, remote access, cameras, and the hardware
specifications for every board.

樹莓派官方文件的完整度很少見，凡是關於 Pi 本身（而不是本專案）的問題，都應該先查這裡。
免費，而且涵蓋的範圍比多數人以為的廣得多 — OS 設定、`config.txt`、編 kernel、遠端連線、
相機、以及每一塊板子的硬體規格。

| Resource 資源 | Link |
|---|---|
| Documentation home 文件首頁 | <https://www.raspberrypi.com/documentation/> |
| Remote access 遠端連線 | <https://www.raspberrypi.com/documentation/computers/remote-access.html> |
| Raspberry Pi Connect | <https://www.raspberrypi.com/documentation/services/connect.html> |
| Community forums 官方論壇 | <https://forums.raspberrypi.com/> |

The documentation site also has an **"Ask a question"** box at the top of the page that answers
from the official documentation, at no cost. It is worth trying before searching the forums,
because it cites the official material rather than a stranger's five-year-old thread.

文件站頂端還有一個 **"Ask a question"** 提問框，會根據官方文件回答，免費。
建議在翻論壇之前先試它 — 它引用的是官方資料，而不是某位路人五年前的舊帖。

---

## Cheat Sheet — 指令速查

| Purpose 用途 | Command 指令 |
|---|---|
| Find the Pi's IP 查 Pi 的 IP | `hostname -I` (uppercase `-I` 大寫) |
| Find the Mac's Wi-Fi IP 查 Mac 的 Wi-Fi IP | `ipconfig getifaddr en0` |
| Open a terminal on the Pi desktop 在 Pi 桌面開終端機 | `Ctrl + Alt + T` |
| Connect over SSH | `ssh <username>@raspberrypi.local` |
| Set up key login 設定金鑰登入 | `ssh-keygen -t ed25519` then `ssh-copy-id <username>@<ip>` |
| Copy a file to the Pi 傳檔到 Pi | `scp <file> <username>@<ip>:` |
| Start a detachable session 開可離線的 session | `tmux new -s carbot` |
| Turn on Pi Connect 啟用 Pi Connect | `rpi-connect on` then `rpi-connect signin` |
| Check Pi Connect state 檢查 Connect 狀態 | `rpi-connect status` |
| Enable SSH / VNC / I2C 開啟介面 | `sudo raspi-config` → `Interface Options` |
| Confirm the NeZha board responds 確認驅動板有回應 | `i2cdetect -y 1` (expects `40`) |
