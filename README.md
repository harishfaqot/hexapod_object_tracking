# 🤖 Hexapod Robot Tracking using YOLOv4-tiny

This project enables a six-legged (hexapod) robot to track and follow objects in real time using the YOLOv4-tiny object detection model. It is optimized for the NVIDIA Jetson Nano and integrates vision, object tracking, and robot locomotion using Dynamixel and PCA9685-controlled servos.

---

## 🚀 Features

- Real-time object detection with YOLOv4-tiny (Darknet)
- Object tracking and direction-based decision making
- Hexapod walking and turning motions
- Supports Dynamixel (serial) and PCA9685 (I2C) servo control
- Optimized for Jetson Nano

---

## 🗂 Project Structure

```
hexapod-tracking-yolov4/
├── cam.py                  # Handles video capture using OpenCV
├── detect.py               # Runs object detection using YOLOv4-tiny
├── hexapod_track.py        # Main tracking loop: detection, tracking, and movement
├── tracking.py             # Target following logic based on bounding boxes
├── lib/
│   ├── hexapod_constant.py # Constants like servo limits, delays, etc.
│   ├── hexapod_control.py  # High-level robot movements (walk, turn)
│   ├── servo.py            # Servo utility functions
│   ├── pca_servo.py        # Control servos via PCA9685 over I2C
├── models/
│   ├── yolov4-tiny.cfg     # YOLOv4-tiny configuration
│   ├── yolov4-tiny.weights # Trained weights
│   ├── coco.names          # Object class labels
├── requirements.txt        # Python package dependencies
├── setup_darknet.sh        # Shell script to set up Darknet with GPU
└── README.md               # Project documentation
```

---

## 🔧 Installation Instructions

### 1. Clone and Build YOLOv4 Darknet

```bash
git clone https://github.com/AlexeyAB/darknet
cd darknet

sed -i 's/OPENCV=0/OPENCV=1/' Makefile
sed -i 's/GPU=0/GPU=1/' Makefile
sed -i 's/CUDNN=0/CUDNN=1/' Makefile
sed -i 's/CUDNN_HALF=0/CUDNN_HALF=1/' Makefile
sed -i 's/LIBSO=0/LIBSO=1/' Makefile
make

export PATH=/usr/local/cuda/bin${PATH:+:${PATH}}
```

### 2. Install Python Dependencies

```bash
pip install -r requirements.txt
```

Or manually:

```bash
pip install dynamixel-sdk Adafruit-PCA9685 opencv-python numpy
```

---

## 🧠 Code Overview

| File | Description |
|------|-------------|
| `hexapod_track.py` | Central control loop: detects objects, tracks targets, and moves the hexapod. |
| `detect.py` | Uses Darknet to detect objects in video frames. |
| `tracking.py` | Computes object center and guides robot movement direction. |
| `cam.py` | Captures frames from the camera using OpenCV. |
| `lib/hexapod_control.py` | Defines robot motions (e.g., walk forward, turn left/right). |
| `lib/hexapod_constant.py` | Movement timings, default angles, servo configurations. |
| `lib/servo.py` | Interfaces with generic servo motors. |
| `lib/pca_servo.py` | Sends PWM commands to PCA9685 for controlling legs. |
| `models/` | Stores YOLOv4-tiny model files: `.cfg`, `.weights`, and `.names`. |

---

## 🧪 Running the Project

1. Ensure the YOLO model files (`.cfg`, `.weights`, `coco.names`) are in the `models/` directory.
2. Connect your hardware (camera, servos, Dynamixel).
3. Run:

```bash
python3 hexapod_track.py
```

---

## 🔌 Hardware Requirements

- NVIDIA Jetson Nano (4GB recommended)
- USB Camera (or CSI camera)
- PCA9685 PWM driver board (I2C)
- Dynamixel servos (AX/MX)
- Power supply for motors (5V/12V depending on servo)
- Optional: OLED or visual indicators

---

## 🐞 Troubleshooting

- **No object detection?** Check if `models/yolov4-tiny.weights` and `.cfg` exist and are valid.
- **Camera not detected?** Try `ls /dev/video*` and update index in `cam.py`.
- **Servo not moving?** Ensure correct port, ID, and power.
- **I2C error?** Confirm address using `sudo i2cdetect -y 1`.

---

## 📜 License

MIT License — free to use and modify.