# Raspberry Pi 官方文件

**唯讀。**

| 檔案 | 說明 |
|---|---|
| `RP-008248-DS-1-bcm2711-peripherals.pdf` | BCM2711 周邊裝置手冊 |

> ⚠️ BCM2711 是 **Raspberry Pi 4** 的 SoC。本專案用的是 **Pi 5（BCM2712）**，
> GPIO 控制器架構不同（Pi 5 的 GPIO 走 RP1 南橋）。這份文件對照 I2C／SPI
> 暫存器語意時仍有參考價值，但**腳位與時脈設定不要直接照抄**。
> Pi 5 的腳位對照見 [`docs/hardware/raspberry-pi-5-pinout.md`](../../docs/hardware/raspberry-pi-5-pinout.md)。

官方文件：<https://www.raspberrypi.com/documentation/computers/raspberry-pi.html>

---

## Part 1: Raspberry Pi 5 Camera Recommendations

### Camera Selection Guide
For AI applications, here are several suitable camera options for Raspberry Pi 5:

1. 🥇 **Raspberry Pi AI Camera (Highly Recommended)**
   This is the top choice for AI applications. It features an integrated Sony IMX500 intelligent vision sensor with an onboard AI accelerator (NPU). Neural network models run directly on the camera hardware, offloading computation from the Raspberry Pi processor. Supports frameworks like TensorFlow and PyTorch, and integrates seamlessly with `rpicam-apps` and `Picamera2`.
   - [AI Camera Documentation](https://www.raspberrypi.com/documentation/accessories/ai-camera.html)

2. **Camera Module 3 + AI HAT+**
   If higher inference performance is required, pair a standard Camera Module 3 (12MP Sony IMX708) with an AI HAT+ or AI HAT+ 2. The AI HAT+ 2 provides up to 40 TOPS of inference performance, ideal for running multiple models concurrently. Note that AI HAT+ is compatible only with Raspberry Pi 5.
   - [AI HAT Documentation](https://www.raspberrypi.com/documentation/accessories/ai-hat.html)

3. **Global Shutter Camera**
   If your AI application involves capturing fast-moving objects (e.g., industrial applications), the Global Shutter Camera is the preferred choice because it captures the entire frame simultaneously without motion blur artifacts.
   - [Camera Documentation](https://www.raspberrypi.com/documentation/accessories/camera.html)

---

## Part 2: Dual Interface & Stereo Vision / 雙接口與立體視覺 (中英文)

### Dual CAM/DISP Ports (Dual MIPI Connectors) / 雙 CAM/DISP 接口 (雙 MIPI 連接器)
Raspberry Pi 5 features two dual-lane CAM/DISP MIPI connectors.
Raspberry Pi 5 確實配備兩個雙通道 CAM/DISP MIPI 連接器。
- [Beginner's Guide](https://www.raspberrypi.com/documentation/computers/raspberry-pi-5.html)

**Q1: Can two different devices be connected simultaneously? / 可以連接兩個不同的裝置嗎？**
- ✅ Yes! You can connect:
  - Two CSI cameras
  - Two DSI displays
  - One camera + One display
- ✅ 可以！你可以：
  - 連接兩個 CSI 相機
  - 連接兩個 DSI 顯示器
  - 或一個相機 + 一個顯示器

**Q2: Can it be used for stereo vision & depth estimation? / 可以用來做立體視覺（左右眼測深度）嗎？**
- ⚠️ **English**: Technically yes, but with important hardware/software limitations: `libcamera` currently does not support hardware stereo camera synchronization. The two cameras must run in separate processes, and 3A (Auto-Exposure, Auto-White Balance, Auto-Focus) parameters cannot be synchronized automatically across sensors. Alternative solutions include external hardware trigger sync (applicable to HQ IMX477 camera modules) or software-based frame timestamp synchronization.
- ⚠️ **中文**：技術上可以同時驅動兩個相機，但有重要限制：`libcamera` 目前不支援硬體立體相機同步。兩個相機必須在獨立的程序中運行，無法跨感測器自動同步 3A（自動曝光、自動白平衡、自動對焦）操作。作為替代方案，可透過外部硬體觸發訊號同步（適用於 HQ IMX477 相機）或軟體時間戳記同步來實現。
- [rpicam-apps Documentation](https://www.raspberrypi.com/documentation/computers/camera_software.html)

**Conclusion / 結論：**
- **English**: While connecting two cameras is supported, precise stereo depth measurement requires additional hardware/software engineering due to frame sync constraints.
- **中文**：雖然可以連接兩個相機，但做精確的立體深度測量會有同步上的挑戰，需要額外的工程處理。

### Summary Recommendation Table / 總結建議對照表

| Requirement / 需求 | Recommendation / 推薦 |
| --- | --- |
| Single Camera AI Vision / AI 視覺（單相機） | Raspberry Pi AI Camera |
| High-Performance AI Inference / 高效能 AI 推理 | Camera Module 3 + AI HAT+ 2 |
| Fast Motion Object Detection / 快速移動物體偵測 | Global Shutter Camera |
| Dual-Camera Application / 雙相機應用 | Supported, but stereo sync requires extra setup / 可行，但立體同步需額外處理 |


