# 3D Camera Control & Calibration Toolkit

A comprehensive RealSense camera control system with calibration, IR emitter management, and image processing capabilities.

##  Workflow Overview

### 1. **Image Capture** → 2. **Calibration** → 3. **Calibrated Usage** → 4. **IR Control**

##  Step-by-Step Usage

### Step 1: Capture Images & Videos
**Use:** `realsense_r200_capture_image.py`
- Take pictures and record videos
- Save camera intrinsics data
- Basic camera functionality testing

### Step 2: Camera Calibration
**Use:** `camera_calibration.py`
- Calibrate camera using checkerboard pattern
- Generates high-quality calibration data
- **Outputs:**
  - `output/calibration_data.pkl` - Complete calibration data
  - `output/calibration_summary.json` - Human-readable summary  
  - `output/camera_matrix.txt` - Camera matrix
  - `calibrated_images/` - Undistorted images

### Step 3: Use Calibrated Camera
**Use:** `use_calibrated_camera.py`
- Load calibration data automatically
- Apply undistortion to live camera feed
- **Note:** Shows purple IR dots when camera device = 0, normal when device = 1

### Step 4: IR Emitter Control
Choose your IR control method:

##  IR Emitter Control Scripts

### `ir_emitter_control.py`
- Shows purple IR dots from emitter
- Basic IR visibility control
- Device switching (0 = dots, 1 = normal)

### `realsense_r200_native.py`
- Runs both normal and dotted modes
- **Save options:**
  - Color images
  - Depth images  
  - Both color + depth
- Native R200 camera control

### `realsense_emitter_control.py`
- Runs both normal and colorful depth windows
- **Full IR Control:**
```
=== REALSENSE IR EMITTER CONTROL ===
Controls:
  '1' - Turn IR emitter OFF
  '2' - Turn IR emitter ON
  '3' - Decrease laser power
  '4' - Increase laser power
  'c' - Show color stream only
  'd' - Show depth stream
  'b' - Show both streams
  's' - Save current frames
  'q' - Quit
```

### `realsense_universal_control.py` (Advanced)
- **Laser Power Control:** Available
- **Enhanced Controls:**
```
Controls:
  '1' - Turn IR emitter OFF
  '2' - Turn IR emitter ON
  '3' - Decrease laser power
  '4' - Increase laser power
  '5' - Adjust exposure (reduce IR visibility)
  '6' - Reset exposure
  'c' - Color stream only
  'd' - Depth stream only
  'b' - Both streams
  's' - Save frames
  'q' - Quit
```
- **Features:** Brightness control + dual window display (normal + colorful depth)

##  Output Structure

```
├── saved_images/           # Captured images
├── saved_videos/          # Recorded videos
├── calibration_images/    # Checkerboard calibration images
├── calibrated_images/     # Undistorted output images
├── output/               # Calibration data files
│   ├── calibration_data.pkl
│   ├── calibration_summary.json
│   └── camera_matrix.txt
└── realsense_captures/   # IR control captures
```

##  Key Controls Summary

| Action | Key | Available In |
|--------|-----|--------------|
| Save image/frame | `s` | All scripts |
| Start/stop video | `r` | Capture scripts |
| IR emitter OFF | `1` | IR control scripts |
| IR emitter ON | `2` | IR control scripts |
| Decrease laser power | `3` | Advanced IR scripts |
| Increase laser power | `4` | Advanced IR scripts |
| Adjust exposure | `5` | Universal control |
| Color stream only | `c` | Multi-stream scripts |
| Depth stream only | `d` | Multi-stream scripts |
| Both streams | `b` | Multi-stream scripts |
| Quit | `q` | All scripts |

##  Understanding IR Dots

The **purple dots** you see are infrared structured light patterns:
- **Purpose:** Enable depth sensing and 3D reconstruction
- **Control:** Can be turned ON/OFF or power adjusted
- **Device behavior:** 
  - Device 0 = Shows IR dots
  - Device 1 = Normal view (no visible dots)

##  Quick Start

1. **Capture test images:**
   ```bash
   python realsense_r200_capture_image.py
   ```

2. **Calibrate your camera:**
   ```bash
   python camera_calibration.py
   ```

3. **Use calibrated camera:**
   ```bash
   python use_calibrated_camera.py
   ```

4. **Control IR emitter:**
   ```bash
   python realsense_universal_control.py
   ```

##  Requirements

```txt
opencv-python>=4.5.0
pyrealsense2>=2.50.0
numpy>=1.21.0
```

##  Notes

- IR dots are normal behavior for depth cameras
- Different scripts offer varying levels of IR control
- Calibration significantly improves image quality
- Save options vary by script (color, depth, or both)
