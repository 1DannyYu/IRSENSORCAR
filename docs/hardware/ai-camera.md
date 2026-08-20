# Raspberry Pi AI Camera (IMX500) Model Guide

> This is the English reference for the camera models installed on the project Raspberry Pi.

Hardware: Raspberry Pi AI Camera with Sony IMX500 intelligent vision sensor (on-sensor NPU).

Sources:
- Model Zoo: <https://github.com/raspberrypi/imx500-models>
- Picamera2 demos: <https://github.com/raspberrypi/picamera2/tree/main/examples/imx500>

---

# Model Guide

## 1.1 Quick Start

```bash
# Install the pre-trained models (already done on this Pi; run only if missing)
sudo apt install imx500-models

# Get the official demo scripts
git clone https://github.com/raspberrypi/picamera2.git
cd picamera2/examples/imx500

# Run a demo, e.g. object detection
python imx500_object_detection_demo.py --model /usr/share/imx500-models/imx500_network_yolo11n_pp.rpk
```

All model files (`*.rpk`) live in `/usr/share/imx500-models/`.

## 1.2 Feature Overview

| Task | What it does | Models |
|---|---|---|
| Image Classification | Classify the whole image into one of 1000 ImageNet classes | 15 |
| Object Detection | Locate and classify multiple objects in a scene (COCO, 80 classes) | 6 |
| Semantic Segmentation | Classify every pixel of the image (PASCAL VOC, 20 classes) | 1 |
| Pose Estimation | Detect human body keypoints / skeleton (COCO KeyPoints) | 1 |

## 1.3 Image Classification

Task: categorize input into predefined classes with a confidence score.
Dataset: ImageNet (1000 classes). Demo script: `imx500_classification_demo.py`

| Model | Top-1 Acc. | Input | Command |
|---|---|---|---|
| EfficientNet-B0 | 72.1% | 224×224 | `python imx500_classification_demo.py --model /usr/share/imx500-models/imx500_network_efficientnet_bo.rpk` |
| EfficientNet Lite-0 | 75.3% | 224×224 | `python imx500_classification_demo.py --model /usr/share/imx500-models/imx500_network_efficientnet_lite0.rpk` |
| EfficientNetV2-B0 | 76.7% | 224×224 | `python imx500_classification_demo.py --model /usr/share/imx500-models/imx500_network_efficientnetv2_b0.rpk` |
| EfficientNetV2-B1 | 77.0% | 240×240 | `python imx500_classification_demo.py --model /usr/share/imx500-models/imx500_network_efficientnetv2_b1.rpk` |
| EfficientNetV2-B2 | 77.7% | 260×260 | `python imx500_classification_demo.py --model /usr/share/imx500-models/imx500_network_efficientnetv2_b2.rpk` |
| MnasNet1.0 | 73.2% | 224×224 | `python imx500_classification_demo.py --model /usr/share/imx500-models/imx500_network_mnasnet1.0.rpk` |
| MobileNetV2 | 71.6% | 224×224 | `python imx500_classification_demo.py --model /usr/share/imx500-models/imx500_network_mobilenet_v2.rpk` |
| MobileViT-XS | 72.3% | 256×256 | `python imx500_classification_demo.py --model /usr/share/imx500-models/imx500_network_mobilevit_xs.rpk` |
| MobileViT-XXS | 67.4% | 256×256 | `python imx500_classification_demo.py --model /usr/share/imx500-models/imx500_network_mobilevit_xxs.rpk` |
| RegNetX-002 | 68.4% | 224×224 | `python imx500_classification_demo.py --model /usr/share/imx500-models/imx500_network_regnetx_002.rpk` |
| RegNetY-002 | 69.4% | 224×224 | `python imx500_classification_demo.py --model /usr/share/imx500-models/imx500_network_regnety_002.rpk` |
| RegNetY-004 | 73.8% | 224×224 | `python imx500_classification_demo.py --model /usr/share/imx500-models/imx500_network_regnety_004.rpk` |
| ResNet-18 | 68.6% | 224×224 | `python imx500_classification_demo.py --model /usr/share/imx500-models/imx500_network_resnet18.rpk` |
| ShuffleNetV2-x1.5 | 72.2% | 224×224 | `python imx500_classification_demo.py --model /usr/share/imx500-models/imx500_network_shufflenet_v2_x1_5.rpk` |
| SqueezeNet-V1.0 | 57.6% | 224×224 | `python imx500_classification_demo.py --model /usr/share/imx500-models/imx500_network_squeezenet1.0.rpk` |

## 1.4 Object Detection

Task: identify and locate multiple objects with bounding boxes.
Dataset: COCO (80 classes). Demo script: `imx500_object_detection_demo.py`

| Model | mAP | Input | Command |
|---|---|---|---|
| YOLO11n (pp*) | 0.374 | 640×640 | `python imx500_object_detection_demo.py --model /usr/share/imx500-models/imx500_network_yolo11n_pp.rpk --bbox-normalization --bbox-order xy` |
| YOLOv8n (pp*) | 0.279 | 640×640 | `python imx500_object_detection_demo.py --model /usr/share/imx500-models/imx500_network_yolov8n_pp.rpk --bbox-normalization --bbox-order xy` |
| EfficientDet Lite-0 (pp*) | 0.252 | 320×320 | `python imx500_object_detection_demo.py --model /usr/share/imx500-models/imx500_network_efficientdet_lite0_pp.rpk` |
| NanoDet Plus | 0.332 | 416×416 | `python imx500_object_detection_demo.py --model /usr/share/imx500-models/imx500_network_nanodet_plus_416x416.rpk` |
| NanoDet Plus (pp*) | 0.320 | 416×416 | `python imx500_object_detection_demo.py --model /usr/share/imx500-models/imx500_network_nanodet_plus_416x416_pp.rpk` |
| SSD MobileNetV2 FPN Lite (pp*) | 0.218 | 320×320 | `python imx500_object_detection_demo.py --model /usr/share/imx500-models/imx500_network_ssd_mobilenetv2_fpnlite_320x320_pp.rpk` |

\* **pp** = post-processing is baked into the network and runs on the IMX500 itself, offloading the CPU.

## 1.5 Semantic Segmentation

Task: assign a category to every pixel of the image.
Dataset: PASCAL VOC (20 classes). Demo script: `imx500_segmentation_demo.py`

| Model | mIOU | Input | Command |
|---|---|---|---|
| DeepLabv3Plus | 0.721 | 320×320 | `python imx500_segmentation_demo.py --model /usr/share/imx500-models/imx500_network_deeplabv3plus.rpk` |

## 1.6 Pose Estimation

Task: detect human body keypoints (head, shoulders, elbows, knees…).
Dataset: COCO KeyPoints. Demo script: `imx500_pose_estimation_higherhrnet_demo.py`

| Model | mAP | Input | Command |
|---|---|---|---|
| HigherHRNet | 0.188 | 288×384 | `python imx500_pose_estimation_higherhrnet_demo.py --model /usr/share/imx500-models/imx500_network_higherhrnet_coco.rpk` |

## 1.7 Notes

1. **`pp` suffix**: post-processing is integrated into the network and executed on the IMX500 edge AI processor; the CPU is almost fully offloaded. Non-`pp` models do the post-processing on the Pi CPU.
2. **`imx500_network_inputtensoronly.rpk`**: debugging model that passes the input tensor through without inference.
3. **`rpk_update_network_intrinsics`**: official tool to update network intrinsics inside an `.rpk` (rarely needed).
4. **Licenses**: models use different open-source licenses (Apache-2.0 / MIT / BSD-3 / AGPL-3.0 / Apple Sample Code). See the official `imx500-models/LICENSES/` directory. Note AGPL-3.0 (YOLO family) before commercial use.
5. **rpicam-apps alternative** (same model files, no Picamera2 demo needed):
   `rpicam-hello -t 0 --model /usr/share/imx500-models/imx500_network_posenet.rpk --post-process-file /usr/share/rpi-camera-assets/imx500_posenet.json`
   (rpicam-apps v1.x installs post-process JSON under `/usr/share/rpi-camera-assets/`; confirm with `dpkg -L rpicam-apps | grep imx500`.)
6. **Typical uses**: pedestrian/vehicle detection (YOLO11n), gesture or object classification (EfficientNet family), obstacle segmentation (DeepLabv3Plus), action recognition (HigherHRNet) — all can be wired into this project's car / robotic arm.

---
