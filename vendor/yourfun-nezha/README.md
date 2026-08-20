# 有方機器人 NeZha（哪吒）總線驅動板

**唯讀。** 要基於這些程式碼改寫，複製到 `src/` 再改，原始檔留著當對照組。

## 來源

| 項目 | 內容 |
|---|---|
| 供應商 | 有方機器人科技有限公司（YOURFUN ROBOTICS） |
| 淘寶店 | <https://shop479988600.taobao.com/> |
| 商品頁 | <https://item.taobao.com/item.htm?id=662768466683> |
| B站教學 | <https://b23.tv/W3a9HJ7> |
| 手冊版本 | V1.0.0（2023-11-27） |
| 驅動庫版本 | V1.0 |
| 取得日期 | 2026-07-29 |

原始完整壓縮包（含 Keil 軟體包、範例 Keil 工程、外設資料）在
`~/Downloads/Nezha哪吒总线方案/`，**沒有整包進 repo**。

## 內容

| 路徑 | 說明 |
|---|---|
| `sdk/stm32/` `sdk/arduino/` `sdk/c51/` | 三個平台的驅動庫原始碼。I2C 協定就是從這裡反推的 |
| `manual/NeZha总线驱动板使用手册.pdf` | 官方使用手冊，25 頁。I2C 協定的權威來源 |
| `wiring/` | 整車與各模組接線圖 |

## 匯入時剔除了什麼

- `1.Keil软件包/`、`1.Arduino IDE软件包/` —— IDE 安裝檔，Windows 專用
- `3.例程/*.zip` —— Keil 專案壓縮包，需要時再從原始下載解開
- `5.外设及其相关资料/` —— PS2 遙控器、K210、巡線感測器資料，目前用不到
- `7.…安装说明书/` 的影片檔
- **組裝說明書 PDF（10.2MB）** —— 內容已經以 50 張頁面圖存在
  [`assets/assembly-guide/pages/`](../../assets/assembly-guide/pages/)，
  網頁版在 [`site/src/pages/assembly-guide/`](../../site/src/pages/assembly-guide/)。
  原始 PDF 留在 `~/Downloads/Nezha哪吒总线方案/哪吒底盘+机械臂安装/安装说明书/`

## 整理過的版本

I2C 協定已整理成 [`docs/hardware/nezha-i2c-protocol.md`](../../docs/hardware/nezha-i2c-protocol.md)，
Python 實作在 [`src/carbot/nezha.py`](../../src/carbot/nezha.py)。**先看那兩份**，
本目錄留作查證用。

## 原始碼編碼

`.c` / `.h` 註解是 **GB18030**，不是 UTF-8。用編輯器開會是亂碼：

```bash
iconv -f GB18030 -t UTF-8 vendor/yourfun-nezha/sdk/stm32/NeZha.h
```
