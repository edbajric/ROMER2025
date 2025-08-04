import cv2
import os
import json
import numpy as np
from datetime import datetime

def save_camera_intrinsics(cap, camera_id, timestamp):
    """Save basic camera intrinsics for calibration"""
    # Get frame properties
    width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
    height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    # Basic intrinsics estimation (you'll need proper calibration later)
    intrinsics_data = {
        "timestamp": timestamp,
        "camera_id": camera_id,
        "resolution": {
            "width": width,
            "height": height
        },
        "fps": fps,
        "estimated_intrinsics": {
            "fx": width * 0.8,  # Rough estimate
            "fy": height * 0.8,  # Rough estimate
            "cx": width / 2,     # Center x
            "cy": height / 2,    # Center y
            "note": "These are estimates - need proper calibration"
        },
        "camera_properties": {
            "brightness": cap.get(cv2.CAP_PROP_BRIGHTNESS),
            "contrast": cap.get(cv2.CAP_PROP_CONTRAST),
            "saturation": cap.get(cv2.CAP_PROP_SATURATION),
            "exposure": cap.get(cv2.CAP_PROP_EXPOSURE)
        }
    }
    
    # Create directory if it doesn't exist
    if not os.path.exists('camera_calibration'):
        os.makedirs('camera_calibration')
    
    filename = f"camera_calibration/camera_{camera_id}_intrinsics_{timestamp}.json"
    with open(filename, 'w') as f:
        json.dump(intrinsics_data, f, indent=2)
    
    print(f"Camera intrinsics saved: {filename}")
    return filename

def save_calibration_image(frame, camera_id, timestamp):
    """Save image for calibration purposes"""
    if not os.path.exists('calibration_images2'):
        os.makedirs('calibration_images2')
    
    filename = f"calibration_images2/calib_camera_{camera_id}_{timestamp}.jpg"
    cv2.imwrite(filename, frame)
    print(f"Calibration image saved: {filename}")
    return filename

device_range = range(4, 10)

print("Each video device will be tested sequentially.")
print("Controls:")
print("  'q' - Move to next device")
print("  's' - Save picture")
print("  'r' - Start/stop recording")
print("  'i' - Save camera intrinsics")
print("  'c' - Save calibration image")

for i in device_range:
    path = f"/dev/video{i}"
    cap = cv2.VideoCapture(path)

    if not cap.isOpened():
        print(f"{path} could not be opened.")
        continue

    print(f"{path} opened. Check if video is coming.")
    print("Controls: q=exit, s=save picture, r=record, i=save intrinsics, c=calibration image")
    
    # Video recording variables
    video_writer = None
    recording = False
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"{path}: frame could not be captured.")
            break
        cv2.imshow(f"Camera: {path}", frame)
        
        # Write frame to video if recording
        if recording and video_writer is not None:
            video_writer.write(frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # Create directory if it doesn't exist
            if not os.path.exists('saved_images'):
                os.makedirs('saved_images')
            
            # Generate filename with timestamp
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"saved_images/camera_{i}_{timestamp}.jpg"
            
            # Save the image
            cv2.imwrite(filename, frame)
            print(f"Picture saved as: {filename}")
        elif key == ord('i'):
            # Save camera intrinsics
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_camera_intrinsics(cap, i, timestamp)
        elif key == ord('c'):
            # Save calibration image
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            save_calibration_image(frame, i, timestamp)
        elif key == ord('r'):
            if not recording:
                # Start recording
                if not os.path.exists('saved_videos'):
                    os.makedirs('saved_videos')
                
                # Generate video filename with timestamp
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                video_filename = f"saved_videos/camera_{i}_{timestamp}.mp4"
                
                # Get frame dimensions
                height, width, _ = frame.shape
                
                # Initialize video writer with MP4 and H.264 codec
                fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                video_writer = cv2.VideoWriter(video_filename, fourcc, 20.0, (width, height))
                
                recording = True
                print(f"Started recording: {video_filename}")
            else:
                # Stop recording
                recording = False
                if video_writer is not None:
                    video_writer.release()
                    video_writer = None
                print("Recording stopped")

    cap.release()
    cv2.destroyAllWindows()