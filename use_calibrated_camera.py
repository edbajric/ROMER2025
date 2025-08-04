# this one also shows the purple dots from IR emitter
# only works with D455, not R200





import cv2
import numpy as np
import pickle
import json
from datetime import datetime
import os

class CalibratedCamera:
    """Class to work with calibrated camera data"""
    
    def __init__(self, calibration_data_path="output/calibration_data.pkl"):
        """Load calibration data"""
        self.loaded = self.load_calibration_data(calibration_data_path)
        if not self.loaded:
            print("Failed to load calibration data. Please check the file path.")
        
    def load_calibration_data(self, pkl_path):
        """Load calibration data from pickle file"""
        try:
            with open(pkl_path, 'rb') as f:
                data = pickle.load(f)
            
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['distortion_coefficients']  # Fixed key name
            self.reprojection_error = data.get('reprojection_error', 'Unknown')
            
            # Extract individual parameters
            self.fx = self.camera_matrix[0, 0]
            self.fy = self.camera_matrix[1, 1] 
            self.cx = self.camera_matrix[0, 2]
            self.cy = self.camera_matrix[1, 2]
            
            print("✓ Calibration data loaded successfully!")
            print(f"  Focal length fx: {self.fx:.2f} pixels")
            print(f"  Focal length fy: {self.fy:.2f} pixels")
            print(f"  Principal point cx: {self.cx:.2f} pixels")
            print(f"  Principal point cy: {self.cy:.2f} pixels")
            print(f"  Reprojection error: {self.reprojection_error}")
            
        except Exception as e:
            print(f"Error loading calibration data: {e}")
            return False
        return True
            
    def undistort_image(self, image):
        """Remove lens distortion from an image"""
        return cv2.undistort(image, self.camera_matrix, self.dist_coeffs)
    
    def pixel_to_camera_coordinates(self, u, v, depth):
        """
        Convert pixel coordinates to 3D camera coordinates
        u, v: pixel coordinates
        depth: depth value in meters
        Returns: (X, Y, Z) in camera coordinate system
        """
        X = (u - self.cx) * depth / self.fx
        Y = (v - self.cy) * depth / self.fy
        Z = depth
        return X, Y, Z
    
    def camera_to_pixel_coordinates(self, X, Y, Z):
        """
        Convert 3D camera coordinates to pixel coordinates
        X, Y, Z: 3D coordinates in camera frame
        Returns: (u, v) pixel coordinates
        """
        u = (X * self.fx / Z) + self.cx
        v = (Y * self.fy / Z) + self.cy
        return int(u), int(v)
    
    def save_calibration_json(self, output_path="output/calibration_summary.json"):
        """Save calibration data in JSON format"""
        calibration_summary = {
            "timestamp": datetime.now().strftime("%Y%m%d_%H%M%S"),
            "camera_intrinsics": {
                "fx": float(self.fx),
                "fy": float(self.fy),
                "cx": float(self.cx),
                "cy": float(self.cy),
                "camera_matrix": self.camera_matrix.tolist(),
                "distortion_coefficients": self.dist_coeffs.tolist()
            },
            "quality": {
                "reprojection_error": self.reprojection_error,
                "notes": "Lower reprojection error means better calibration"
            },
            "usage": {
                "3d_reconstruction": "Use for converting pixels to 3D coordinates",
                "measurement": "Use for accurate distance/size measurements",
                "computer_vision": "Use for any CV application requiring precision"
            }
        }
        
        with open(output_path, 'w') as f:
            json.dump(calibration_summary, f, indent=2)
        
        print(f"Calibration summary saved to: {output_path}")

def find_working_camera():
    """Find a working camera device and detect if it's R200"""
    print("Searching for working camera devices...")
    
    for i in range(10):  # Check /dev/video0 to /dev/video9
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            # Test if we can actually read a frame
            ret, frame = cap.read()
            if ret:
                # Check if this might be R200 (common issues: green tint, unusual aspect ratio)
                height, width = frame.shape[:2]
                is_r200_likely = False
                
                # R200 often has greenish tint or unusual format
                if len(frame.shape) == 3:
                    # Check for green dominance (R200 color format issue)
                    green_mean = np.mean(frame[:,:,1])
                    red_mean = np.mean(frame[:,:,2])
                    blue_mean = np.mean(frame[:,:,0])
                    
                    if green_mean > red_mean * 1.3 and green_mean > blue_mean * 1.3:
                        is_r200_likely = True
                        print(f"✓ Camera found at device {i} (R200 detected - green tint)")
                    else:
                        print(f"✓ Working camera found at device {i}")
                else:
                    print(f"✓ Working camera found at device {i}")
                
                cap.release()
                return i, is_r200_likely
            cap.release()
    
    print("✗ No working camera devices found")
    return None, False

def live_camera_with_calibration():
    """Live camera feed with undistortion and measurement capabilities"""
    
    # Load calibrated camera
    cal_cam = CalibratedCamera()
    if not cal_cam.loaded:
        return
    
    # Find working camera device
    camera_result = find_working_camera()
    if camera_result[0] is None:
        print("No camera available. Please check your camera connection.")
        return
    
    camera_device, is_r200 = camera_result
    
    # Open camera
    cap = cv2.VideoCapture(camera_device)
    
    if not cap.isOpened():
        print(f"Could not open camera device {camera_device}")
        return
    
    # R200-specific settings
    if is_r200:
        print("🔧 Applying R200-specific settings...")
        # R200 works better with these settings
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_FPS, 30)
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        cap.set(cv2.CAP_PROP_EXPOSURE, -6)  # Lower exposure for R200
        cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)
        cap.set(cv2.CAP_PROP_CONTRAST, 32)
        cap.set(cv2.CAP_PROP_SATURATION, 64)  # Reduce saturation to fix green tint
        cap.set(cv2.CAP_PROP_GAIN, 0)
    else:
        # Standard camera settings
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
        cap.set(cv2.CAP_PROP_BRIGHTNESS, 50)
    
    print("Live Calibrated Camera Feed")
    if is_r200:
        print("📷 R200 Mode: Color correction enabled")
    print("Controls:")
    print("  's' - Save undistorted image")
    print("  'm' - Measure distance between two points (click two points)")
    print("  'u' - Toggle undistortion on/off")
    print("  'c' - Toggle color correction (R200)")
    print("  'q' - Quit")
    
    show_undistorted = True
    measuring = False
    measure_points = []
    color_correction = is_r200  # Enable by default for R200
    
    def mouse_callback(event, x, y, flags, param):
        nonlocal measuring, measure_points
        
        if event == cv2.EVENT_LBUTTONDOWN and measuring:
            measure_points.append((x, y))
            print(f"Point {len(measure_points)}: ({x}, {y})")
            
            if len(measure_points) == 2:
                # Calculate distance (assuming both points are on same plane)
                # For real measurement, you need depth information
                p1, p2 = measure_points
                pixel_distance = np.sqrt((p2[0] - p1[0])**2 + (p2[1] - p1[1])**2)
                print(f"Pixel distance: {pixel_distance:.2f} pixels")
                
                # Reset for next measurement
                measure_points = []
                measuring = False
                print("Measurement complete. Press 'm' for another measurement.")
    
    def fix_r200_colors(frame):
        """Fix R200 color issues - green tint and format problems"""
        if len(frame.shape) != 3:
            return frame
            
        # Method 1: Reduce green channel dominance
        corrected = frame.copy()
        corrected[:,:,1] = corrected[:,:,1] * 0.7  # Reduce green
        corrected[:,:,0] = corrected[:,:,0] * 1.1  # Boost blue slightly
        corrected[:,:,2] = corrected[:,:,2] * 1.1  # Boost red slightly
        
        # Method 2: White balance correction
        # Calculate channel means
        b_mean = np.mean(corrected[:,:,0])
        g_mean = np.mean(corrected[:,:,1])
        r_mean = np.mean(corrected[:,:,2])
        
        # Target gray value
        gray_target = (b_mean + g_mean + r_mean) / 3
        
        # Apply white balance
        if b_mean > 0:
            corrected[:,:,0] = corrected[:,:,0] * (gray_target / b_mean)
        if g_mean > 0:
            corrected[:,:,1] = corrected[:,:,1] * (gray_target / g_mean)
        if r_mean > 0:
            corrected[:,:,2] = corrected[:,:,2] * (gray_target / r_mean)
        
        # Ensure values stay in valid range
        corrected = np.clip(corrected, 0, 255).astype(np.uint8)
        
        return corrected
    
    cv2.namedWindow('Calibrated Camera', cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback('Calibrated Camera', mouse_callback)
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Fix R200 color issues if needed
        if color_correction and is_r200:
            frame = fix_r200_colors(frame)
        
        # Apply undistortion if enabled
        display_frame = frame.copy()
        if show_undistorted:
            display_frame = cal_cam.undistort_image(frame)
        
        # Draw measurement points
        for i, point in enumerate(measure_points):
            cv2.circle(display_frame, point, 5, (0, 255, 0), -1)
            cv2.putText(display_frame, f"P{i+1}", (point[0]+10, point[1]-10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
        # Draw status text
        status_text = f"Undistortion: {'ON' if show_undistorted else 'OFF'}"
        if is_r200:
            status_text += f" | R200 Color Fix: {'ON' if color_correction else 'OFF'}"
        if measuring:
            status_text += " | MEASURING: Click 2 points"
        
        cv2.putText(display_frame, status_text, (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        
        # Add camera info
        info_text = f"Device {camera_device} | {display_frame.shape[1]}x{display_frame.shape[0]}"
        cv2.putText(display_frame, info_text, (10, display_frame.shape[0] - 10), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        cv2.imshow('Calibrated Camera', display_frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('s'):
            # Save undistorted image
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            if not os.path.exists('calibrated_images'):
                os.makedirs('calibrated_images')
            
            # Apply color correction before undistortion if needed
            save_frame = frame.copy()
            if color_correction and is_r200:
                save_frame = fix_r200_colors(save_frame)
            
            undistorted = cal_cam.undistort_image(save_frame)
            filename = f"calibrated_images/undistorted_{timestamp}.jpg"
            cv2.imwrite(filename, undistorted)
            print(f"Undistorted image saved: {filename}")
            
        elif key == ord('u'):
            show_undistorted = not show_undistorted
            print(f"Undistortion {'enabled' if show_undistorted else 'disabled'}")
            
        elif key == ord('c') and is_r200:
            color_correction = not color_correction
            print(f"R200 color correction {'enabled' if color_correction else 'disabled'}")
            
        elif key == ord('m'):
            measuring = True
            measure_points = []
            print("Click two points to measure distance")
    
    cap.release()
    cv2.destroyAllWindows()

def analyze_calibration_quality():
    """Analyze the quality of your calibration"""
    print("=== CALIBRATION QUALITY ANALYSIS ===")
    
    # Load calibration data
    cal_cam = CalibratedCamera()
    
    print(f"\\nYour Camera Parameters:")
    print(f"Resolution: 640x480 (assumed)")
    print(f"Focal length: {cal_cam.fx:.1f} x {cal_cam.fy:.1f} pixels")
    print(f"Field of view (approx): {2 * np.arctan(320/cal_cam.fx) * 180/np.pi:.1f}° horizontal")
    print(f"Principal point: ({cal_cam.cx:.1f}, {cal_cam.cy:.1f})")
    
    # Quality assessment
    print(f"\\nQuality Assessment:")
    if isinstance(cal_cam.reprojection_error, (int, float)):
        error = float(cal_cam.reprojection_error)
        if error < 0.5:
            print(f"✓ Excellent calibration (error: {error:.3f} pixels)")
        elif error < 1.0:
            print(f"✓ Good calibration (error: {error:.3f} pixels)")
        elif error < 2.0:
            print(f"⚠ Fair calibration (error: {error:.3f} pixels)")
        else:
            print(f"✗ Poor calibration (error: {error:.3f} pixels) - consider recalibrating")
    
    # What you can do now
    print(f"\\nWhat You Can Do Now:")
    print("1. Use live_camera_with_calibration() for real-time undistorted feed")
    print("2. Undistort any image using cal_cam.undistort_image()")
    print("3. Convert pixel coordinates to 3D (when you have depth)")
    print("4. Accurate measurements in computer vision applications")
    
    # Save summary
    cal_cam.save_calibration_json()

if __name__ == "__main__":
    print("=== CALIBRATED CAMERA TOOLKIT ===")
    print()
    print("Choose an option:")
    print("1. Analyze calibration quality")
    print("2. Live camera with undistortion")
    print("3. Load calibration data only")
    print()
    
    choice = input("Enter choice (1-3): ").strip()
    
    if choice == "1":
        analyze_calibration_quality()
    elif choice == "2":
        live_camera_with_calibration()
    elif choice == "3":
        cal_cam = CalibratedCamera()
        print("Calibration data loaded. Use cal_cam object for operations.")
    else:
        print("Invalid choice")
