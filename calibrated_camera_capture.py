import cv2
import os
import json
import numpy as np
import pickle
from datetime import datetime

class CalibratedCamera:
    """Load and use calibration data"""
    def __init__(self):
        try:
            with open('output/calibration_data.pkl', 'rb') as f:
                data = pickle.load(f)
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['distortion_coefficients']
            self.loaded = True
            print("✓ Calibration loaded! (fx: {:.1f}, fy: {:.1f})".format(
                self.camera_matrix[0,0], self.camera_matrix[1,1]))
        except:
            self.loaded = False
            print("No calibration data found")
    
    def undistort_image(self, image):
        if self.loaded:
            return cv2.undistort(image, self.camera_matrix, self.dist_coeffs)
        return image

# Load calibration
cal_cam = CalibratedCamera()

device_range = range(0, 10)  # Check all devices from 0-9
print("Each video device will be tested sequentially.")
print("Controls:")
print("  'q' - Move to next device")
print("  's' - Save picture")
print("  'r' - Start/stop recording")
print("  'u' - Save undistorted image (if calibrated)")
print("  't' - Toggle real-time undistortion display")
print("  'c' - Save calibration image")

for i in device_range:
    path = f"/dev/video{i}"
    cap = cv2.VideoCapture(path)

    if not cap.isOpened():
        print(f"{path} could not be opened.")
        continue

    print(f"{path} opened. Calibration: {'✓' if cal_cam.loaded else '✗'}")
    
    # Video recording variables
    video_writer = None
    recording = False
    show_undistorted = False
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print(f"{path}: frame could not be captured.")
            break
        
        # Choose which frame to display
        display_frame = frame.copy()
        if show_undistorted and cal_cam.loaded:
            display_frame = cal_cam.undistort_image(frame)
        
        # Add status text
        status = f"Device: {i} | Undistorted: {'ON' if show_undistorted else 'OFF'}"
        cv2.putText(display_frame, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        cv2.imshow(f"Camera: {path}", display_frame)
        
        # Write frame to video if recording
        if recording and video_writer is not None:
            # Record the undistorted version if calibration is available
            record_frame = cal_cam.undistort_image(frame) if cal_cam.loaded else frame
            video_writer.write(record_frame)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('s'):
            # Save regular image
            if not os.path.exists('saved_images'):
                os.makedirs('saved_images')
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"saved_images/camera_{i}_{timestamp}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Picture saved: {filename}")
            
        elif key == ord('u') and cal_cam.loaded:
            # Save undistorted image
            if not os.path.exists('calibrated_images'):
                os.makedirs('calibrated_images')
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            undistorted = cal_cam.undistort_image(frame)
            filename = f"calibrated_images/undistorted_{i}_{timestamp}.jpg"
            cv2.imwrite(filename, undistorted)
            print(f"Undistorted image saved: {filename}")
            
        elif key == ord('t'):
            # Toggle undistortion display
            if cal_cam.loaded:
                show_undistorted = not show_undistorted
                print(f"Undistortion display: {'ON' if show_undistorted else 'OFF'}")
            else:
                print("No calibration data available")
                
        elif key == ord('c'):
            # Save calibration image
            if not os.path.exists('calibration_images'):
                os.makedirs('calibration_images')
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"calibration_images/calib_camera_{i}_{timestamp}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Calibration image saved: {filename}")
            
        elif key == ord('r'):
            if not recording:
                # Start recording
                if not os.path.exists('saved_videos'):
                    os.makedirs('saved_videos')
                
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                suffix = "_calibrated" if cal_cam.loaded else ""
                video_filename = f"saved_videos/camera_{i}_{timestamp}{suffix}.mp4"
                
                # Get frame dimensions
                height, width, _ = frame.shape
                
                # Initialize video writer
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
