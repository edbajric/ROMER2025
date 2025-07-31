# 3D Cameras Project

This project contains scripts for working with RealSense and other 3D cameras.

## Files

- `realsense_r200.py` - Camera testing script with image capture and video recording
- `realsense_zr300.py` - Additional RealSense camera script

## Usage

Run the camera test script:
```bash
python realsense_r200.py
```

Controls:
- `q` - Move to next camera device
- `s` - Save current frame as image
- `r` - Start/stop video recording

## Output

- Images are saved to `saved_images/` directory
- Videos are saved to `saved_videos/` directory

## Requirements

- OpenCV (cv2)
- Python 3.x
