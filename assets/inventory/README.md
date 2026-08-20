# Inventory Assets and Camera Hardware Selection

This directory stores physical photo assets and hardware documentation for project modules.

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

## Part 2: Dual Interface and Stereo Vision

### Dual CAM/DISP Ports (Dual MIPI Connectors)
Raspberry Pi 5 features two dual-lane CAM/DISP MIPI connectors.
- [Beginner's Guide](https://www.raspberrypi.com/documentation/computers/raspberry-pi-5.html)

**Q1: Can two different devices be connected simultaneously?**
- ✅ Yes! You can connect:
  - Two CSI cameras
  - Two DSI displays
  - One camera + One display
**Q2: Can it be used for stereo vision and depth estimation?**
- ⚠️ Technically yes, but with important hardware/software limitations: `libcamera` currently does not support hardware stereo camera synchronization. The two cameras must run in separate processes, and 3A (Auto-Exposure, Auto-White Balance, Auto-Focus) parameters cannot be synchronized automatically across sensors. Alternative solutions include external hardware trigger sync (applicable to HQ IMX477 camera modules) or software-based frame timestamp synchronization.
- [rpicam-apps Documentation](https://www.raspberrypi.com/documentation/computers/camera_software.html)

**Conclusion:** While connecting two cameras is supported, precise stereo depth measurement
requires additional hardware/software engineering due to frame-sync constraints.

### Summary Recommendation Table

| Requirement | Recommendation |
| --- | --- |
| Single-camera AI vision | Raspberry Pi AI Camera |
| High-performance AI inference | Camera Module 3 + AI HAT+ 2 |
| Fast-motion object detection | Global Shutter Camera |
| Dual-camera application | Supported, but stereo sync requires extra setup |
