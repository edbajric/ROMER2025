import cv2
import numpy as np
import os
import json
import shutil
from datetime import datetime
import pyrealsense2 as rs

class RGBDCaptureSession:
    """Capture and organize RGB-D data into session folders"""
    
    def __init__(self):
        self.session_dir = None
        self.setup_session()
        self.pipeline = None
        self.config = None
        self.device_model = None
        
    def setup_session(self):
        """Create timestamped session directory"""
        main_dir = "RGBD_photos"
        os.makedirs(main_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(main_dir, f"session_{timestamp}")
        os.makedirs(self.session_dir, exist_ok=True)
        
        print(f" Created session: {self.session_dir}")
        
    def detect_realsense_model(self):
        """Detect which RealSense camera model is connected"""
        try:
            ctx = rs.context()
            devices = ctx.query_devices()
            
            if len(devices) == 0:
                return None, None
            
            device = devices[0]
            name = device.get_info(rs.camera_info.name)
            
            if "R200" in name:
                return device, "R200"
            elif "D435" in name or "D455" in name or "D415" in name:
                return device, "D4xx"
            elif "SR300" in name:
                return device, "SR300"
            else:
                return device, "Unknown"
        except:
            return None, None
    
    def initialize_realsense(self):
        """Initialize RealSense camera with model-specific settings"""
        device, model = self.detect_realsense_model()
        
        if not device:
            print(" No RealSense devices found!")
            return False
        
        self.device_model = model
        print(f"✓ Detected: {device.get_info(rs.camera_info.name)} ({model})")
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        try:
            if model == "R200":
                # R200 specific configuration
                self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
                self.config.enable_stream(rs.stream.depth, 628, 468, rs.format.z16, 30)
            else:
                # Modern RealSense configuration
                self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
            
            self.profile = self.pipeline.start(self.config)
            print("✓ RealSense pipeline started")
            return True
            
        except Exception as e:
            print(f" Failed to start RealSense: {e}")
            return False
    
    def initialize_opencv_fallback(self):
        """Fallback to OpenCV for R200 if RealSense SDK fails"""
        print("Trying OpenCV fallback for R200...")
        
        self.color_cap = None
        self.depth_cap = None
        self.device_model = "R200_OpenCV"
        
        # Try to find color and depth streams
        for i in range(6):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and len(frame.shape) == 3:  # Color image
                    if self.color_cap is None:
                        print(f"✓ Color stream found on /dev/video{i}")
                        self.color_cap = cap
                        continue
                    elif self.depth_cap is None:
                        print(f"✓ Depth stream found on /dev/video{i}")
                        self.depth_cap = cap
                        break
                cap.release()
        
        return self.color_cap is not None or self.depth_cap is not None
    
    def get_frames_realsense(self):
        """Get frames from RealSense camera"""
        try:
            frames = self.pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return None, None
            
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Handle R200 color format
            if self.device_model == "R200" and len(color_image.shape) == 3:
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
            
            return color_image, depth_image
            
        except Exception as e:
            print(f"Frame error: {e}")
            return None, None
    
    def get_frames_opencv(self):
        """Get frames from OpenCV (fallback)"""
        color_frame = None
        depth_frame = None
        
        if self.color_cap:
            ret, color_frame = self.color_cap.read()
            if not ret:
                color_frame = None
        
        if self.depth_cap:
            ret, depth_frame = self.depth_cap.read()
            if not ret:
                depth_frame = None
        
        return color_frame, depth_frame
    
    def save_rgbd_triplet(self, color_image, depth_image):
        """Save RGB-D triplet (color, depth, metadata) in session folder"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # Include milliseconds
        
        # Save color image
        color_filename = f"color_{timestamp}.jpg"
        color_path = os.path.join(self.session_dir, color_filename)
        cv2.imwrite(color_path, color_image)
        
        # Save depth image (16-bit PNG to preserve depth precision)
        depth_filename = f"depth_{timestamp}.png"
        depth_path = os.path.join(self.session_dir, depth_filename)
        cv2.imwrite(depth_path, depth_image)
        
        # Create metadata JSON
        metadata = {
            "timestamp": timestamp,
            "session_dir": self.session_dir,
            "color_image": color_filename,  # Relative path within session
            "depth_image": depth_filename,  # Relative path within session
            "camera_model": self.device_model,
            "color_format": "BGR" if self.device_model != "R200" else "RGB",
            "depth_format": "16-bit PNG",
            "notes": f"RGB-D triplet captured with {self.device_model}"
        }
        
        json_filename = f"rgbd_{timestamp}.json"
        json_path = os.path.join(self.session_dir, json_filename)
        
        with open(json_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        print(f"✓ RGB-D triplet saved: {timestamp}")
        return {
            'color': color_path,
            'depth': depth_path,
            'metadata': json_path
        }
    
    def create_session_summary(self, captured_count):
        """Create session summary"""
        summary = {
            "session_timestamp": datetime.now().strftime("%Y%m%d_%H%M%S"),
            "session_directory": self.session_dir,
            "camera_model": self.device_model,
            "total_captures": captured_count,
            "capture_format": "RGB-D triplets (color.jpg + depth.png + metadata.json)",
            "ready_for_conversion": True,
            "next_steps": [
                f"Run: python convert_session_to_pointclouds.py {self.session_dir}",
                f"Or: meshlab {self.session_dir}/pointclouds/*.ply"
            ]
        }
        
        summary_path = os.path.join(self.session_dir, "session_summary.json")
        with open(summary_path, 'w') as f:
            json.dump(summary, f, indent=2)
        
        print(f" Session summary saved: {summary_path}")
    
    def run_capture_session(self):
        """Main capture loop"""
        # Try RealSense SDK first
        if not self.initialize_realsense():
            # Fallback to OpenCV
            if not self.initialize_opencv_fallback():
                print(" No camera initialization method worked!")
                return
        
        print(f"\n RGB-D Capture Session Started")
        print(f"Session folder: {self.session_dir}")
        print(f" Camera: {self.device_model}")
        print("\nControls:")
        print("  's' - Save RGB-D triplet")
        print("  'q' - Quit and finalize session")
        
        capture_count = 0
        
        while True:
            # Get frames based on method
            if self.pipeline:
                color_image, depth_image = self.get_frames_realsense()
            else:
                color_image, depth_image = self.get_frames_opencv()
            
            # Display color frame
            if color_image is not None:
                display_frame = color_image.copy()
                
                # Add overlay info
                cv2.putText(display_frame, f"Session: {os.path.basename(self.session_dir)}", 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(display_frame, f"Captures: {capture_count}", 
                           (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(display_frame, f"Camera: {self.device_model}", 
                           (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                cv2.imshow('RGB-D Capture Session', display_frame)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('s'):
                if color_image is not None and depth_image is not None:
                    self.save_rgbd_triplet(color_image, depth_image)
                    capture_count += 1
                else:
                    print(" Cannot save - missing color or depth frame")
        
        # Finalize session
        self.create_session_summary(capture_count)
        print(f"\n Session completed!")
        print(f"Captured {capture_count} RGB-D triplets")
        print(f" Saved to: {self.session_dir}")
        
        if capture_count > 0:
            print(f"\n Next steps:")
            print(f"1. Convert to point clouds: python convert_session_to_pointclouds.py {self.session_dir}")
            print(f"2. View in MeshLab: meshlab {self.session_dir}/pointclouds/*.ply")
        
        # Cleanup
        self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        if self.pipeline:
            try:
                self.pipeline.stop()
            except:
                pass
        
        if hasattr(self, 'color_cap') and self.color_cap:
            self.color_cap.release()
        if hasattr(self, 'depth_cap') and self.depth_cap:
            self.depth_cap.release()
        
        cv2.destroyAllWindows()

def main():
    print("=== RGB-D CAPTURE & ORGANIZE SESSION ===")
    print("Captures RGB-D data and organizes into timestamped session folders")
    print("Supports RealSense D455, R200, and OpenCV fallback")
    print()
    
    session = RGBDCaptureSession()
    session.run_capture_session()

if __name__ == "__main__":
    main()