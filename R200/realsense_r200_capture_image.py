import cv2
import os
import json
import numpy as np
from datetime import datetime

def check_available_devices():
    """Check which video devices actually exist"""
    print("Checking available video devices...")
    available_devices = []
    
    for i in range(20):  # Check more devices
        device_path = f"/dev/video{i}"
        if os.path.exists(device_path):
            print(f"  Found: {device_path}")
            available_devices.append(i)
        else:
            # Stop after finding 5 consecutive missing devices
            if i > 5 and len(available_devices) == 0:
                break
    
    if not available_devices:
        print(" No video devices found!")
        print(" Try:")
        print("   - Connect a camera (webcam, USB camera)")
        print("   - Run: ls -la /dev/video*")
        print("   - Check: v4l2-ctl --list-devices")
        return []
    
    print(f" Available devices: {available_devices}")
    return available_devices

def test_device_capability(device_id):
    """Test if a device can actually capture frames"""
    device_path = f"/dev/video{device_id}"
    print(f"\nTesting {device_path}...")
    
    cap = cv2.VideoCapture(device_id)
    
    if not cap.isOpened():
        print(f"{device_path} - Could not open")
        return False
    
    # Get device info
    width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
    height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
    fps = cap.get(cv2.CAP_PROP_FPS)
    
    print(f" Resolution: {int(width)}x{int(height)}")
    print(f" FPS: {fps}")
    
    # Test frame capture
    ret, frame = cap.read()
    
    if not ret or frame is None:
        print(f" {device_path} - Could not capture frame")
        cap.release()
        return False
    
    print(f"{device_path} - Frame captured successfully ({frame.shape})")
    cap.release()
    return True

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

def main():
    print("=== CAMERA DEVICE SCANNER & CAPTURE ===")
    
    # Step 1: Check what devices exist
    available_devices = check_available_devices()
    
    if not available_devices:
        print("\n Quick fixes to try:")
        print("   1. Connect a USB camera/webcam")
        print("   2. Check permissions: sudo chmod 666 /dev/video*")
        print("   3. List devices: ls -la /dev/video*")
        return
    
    # Step 2: Test each device
    working_devices = []
    for device_id in available_devices:
        if test_device_capability(device_id):
            working_devices.append(device_id)
    
    if not working_devices:
        print("\n No working camera devices found!")
        return
    
    print(f"\n Working devices: {working_devices}")
    print("\nControls for each device:")
    print("  'q' - Move to next device")
    print("  's' - Save picture")
    print("  'r' - Start/stop recording")
    print("  'i' - Save camera intrinsics")
    print("  'c' - Save calibration image")
    
    # Step 3: Capture from each working device
    for device_id in working_devices:
        print(f"\nOpening camera /dev/video{device_id}...")
        
        cap = cv2.VideoCapture(device_id)
        
        if not cap.isOpened():
            print(f"Failed to open /dev/video{device_id}")
            continue
        
        print(f"Camera /dev/video{device_id} opened successfully")
        print("   Press 'q' to move to next camera")
        
        # Video recording variables
        video_writer = None
        recording = False
        frame_count = 0
        
        while True:
            ret, frame = cap.read()
            if not ret:
                print(f"Failed to capture frame from /dev/video{device_id}")
                break
            
            frame_count += 1
            
            # Add device info overlay
            cv2.putText(frame, f"Device: /dev/video{device_id}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(frame, f"Frames: {frame_count}", (10, 60), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            if recording:
                cv2.putText(frame, "RECORDING", (10, 90), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            cv2.imshow(f"Camera /dev/video{device_id}", frame)
            
            # Write frame to video if recording
            if recording and video_writer is not None:
                video_writer.write(frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                print(f"Moving to next device...")
                break
            elif key == ord('s'):
                # Save picture
                if not os.path.exists('saved_images'):
                    os.makedirs('saved_images')
                
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"saved_images/camera_{device_id}_{timestamp}.jpg"
                
                cv2.imwrite(filename, frame)
                print(f" Picture saved: {filename}")
                
            elif key == ord('i'):
                # Save camera intrinsics
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_camera_intrinsics(cap, device_id, timestamp)
                
            elif key == ord('c'):
                # Save calibration image
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_calibration_image(frame, device_id, timestamp)
                
            elif key == ord('r'):
                if not recording:
                    # Start recording
                    if not os.path.exists('saved_videos'):
                        os.makedirs('saved_videos')
                    
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    video_filename = f"saved_videos/camera_{device_id}_{timestamp}.mp4"
                    
                    height, width = frame.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    video_writer = cv2.VideoWriter(video_filename, fourcc, 20.0, (width, height))
                    
                    if video_writer.isOpened():
                        recording = True
                        print(f" Started recording: {video_filename}")
                    else:
                        print("Failed to start recording")
                        video_writer = None
                else:
                    # Stop recording
                    recording = False
                    if video_writer is not None:
                        video_writer.release()
                        video_writer = None
                    print(" Recording stopped")
        
        # Cleanup
        cap.release()
        if video_writer is not None:
            video_writer.release()
        cv2.destroyAllWindows()
    
    print("\nCamera scanning complete!")

if __name__ == "__main__":
    main()