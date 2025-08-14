import cv2
import numpy as np
import os
import json
from datetime import datetime
import subprocess
import time

class ZR300RGBDCaptureSession:
    """ZR300-specific RGB-D capture using OpenCV and V4L2"""
    
    def __init__(self):
        self.session_dir = None
        self.setup_session()
        self.color_cap = None
        self.depth_cap = None
        self.color_device = None
        self.depth_device = None
        
    def setup_session(self):
        """Create timestamped session directory"""
        main_dir = "RGBD_photos"
        os.makedirs(main_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(main_dir, f"session_{timestamp}")
        os.makedirs(self.session_dir, exist_ok=True)
        
        print(f" Created session: {self.session_dir}")
        
    def detect_zr300_devices(self):
        """Detect ZR300 video devices using v4l2"""
        print("Scanning for RealSense ZR300 devices...")
        
        color_candidates = []
        depth_candidates = []
        
        # Check /dev/video* devices
        for i in range(10):
            device_path = f"/dev/video{i}"
            if os.path.exists(device_path):
                try:
                    # Get device info using v4l2-ctl
                    result = subprocess.run(
                        ['v4l2-ctl', '-d', device_path, '--info'], 
                        capture_output=True, text=True, timeout=3
                    )
                    
                    if "Intel" in result.stdout and ("RealSense" in result.stdout or "ZR300" in result.stdout):
                        # Get supported formats
                        formats_result = subprocess.run(
                            ['v4l2-ctl', '-d', device_path, '--list-formats-ext'], 
                            capture_output=True, text=True, timeout=3
                        )
                        
                        formats_output = formats_result.stdout
                        
                        # Determine if this is color or depth
                        if "YUYV" in formats_output or "RGB" in formats_output:
                            color_candidates.append(i)
                            print(f"  Color stream candidate: /dev/video{i}")
                        elif "Y16" in formats_output or "GREY" in formats_output:
                            depth_candidates.append(i)
                            print(f"  Depth stream candidate: /dev/video{i}")
                        else:
                            print(f"  Unknown stream: /dev/video{i}")
                            
                except (subprocess.TimeoutExpired, subprocess.CalledProcessError, FileNotFoundError):
                    # v4l2-ctl not available or device busy, try OpenCV probe
                    cap = cv2.VideoCapture(i)
                    if cap.isOpened():
                        ret, frame = cap.read()
                        if ret:
                            if len(frame.shape) == 3 and frame.shape[2] == 3:
                                color_candidates.append(i)
                                print(f"  Color stream (OpenCV): /dev/video{i}")
                            elif len(frame.shape) == 2 or (len(frame.shape) == 3 and np.mean(frame) < 50):
                                depth_candidates.append(i)
                                print(f"  Depth stream (OpenCV): /dev/video{i}")
                        cap.release()
        
        return color_candidates, depth_candidates
    
    def setup_zr300_streams(self):
        """Setup ZR300 color and depth streams"""
        color_candidates, depth_candidates = self.detect_zr300_devices()
        
        if not color_candidates and not depth_candidates:
            print("No ZR300 devices detected!")
            print("Make sure:")
            print("  - ZR300 is connected via USB 3.0")
            print("  - No other software is using the camera")
            print("  - Try: sudo chmod 666 /dev/video*")
            return False
        
        # Setup color stream
        if color_candidates:
            for device_id in color_candidates:
                try:
                    cap = cv2.VideoCapture(device_id)
                    if cap.isOpened():
                        # ZR300 color stream settings
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        cap.set(cv2.CAP_PROP_FPS, 30)
                        # Test capture
                        ret, frame = cap.read()
                        if ret and len(frame.shape) == 3:
                            self.color_cap = cap
                            self.color_device = device_id
                            print(f"Color stream ready: /dev/video{device_id}")
                            break
                        cap.release()
                except Exception as e:
                    print(f"Failed to setup color on /dev/video{device_id}: {e}")

        # Setup depth stream
        if depth_candidates:
            for device_id in depth_candidates:
                if device_id == self.color_device:
                    continue  # Skip if already used for color
                try:
                    cap = cv2.VideoCapture(device_id)
                    if cap.isOpened():
                        # ZR300 depth stream settings
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)  # ZR300 native depth resolution
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        cap.set(cv2.CAP_PROP_FPS, 30)
                        # Test capture
                        ret, frame = cap.read()
                        if ret:
                            self.depth_cap = cap
                            self.depth_device = device_id
                            print(f"Depth stream ready: /dev/video{device_id}")
                            break
                        cap.release()
                except Exception as e:
                    print(f"Failed to setup depth on /dev/video{device_id}: {e}")
        
        success = self.color_cap is not None or self.depth_cap is not None
        if success:
            print(f"\nZR300 Setup Complete:")
            print(f"  Color: /dev/video{self.color_device if self.color_device is not None else 'N/A'}")
            print(f"  Depth: /dev/video{self.depth_device if self.depth_device is not None else 'N/A'}")
        return success
    
    def get_zr300_frames(self):
        color_frame = None
        depth_frame = None

        # Get color frame
        if self.color_cap:
            ret, color_frame = self.color_cap.read()
            if ret:
                # Only convert if YUYV (2 channels)
                if len(color_frame.shape) == 2 or (len(color_frame.shape) == 3 and color_frame.shape[2] == 2):
                    color_frame = cv2.cvtColor(color_frame, cv2.COLOR_YUV2BGR_YUYV)
                # else: already BGR, do nothing
            else:
                color_frame = None

        # Get depth frame
        if self.depth_cap:
            ret, depth_frame = self.depth_cap.read()
            if ret:
                # If depth is 3-channel, convert to grayscale
                if len(depth_frame.shape) == 3 and depth_frame.shape[2] == 3:
                    depth_frame = cv2.cvtColor(depth_frame, cv2.COLOR_BGR2GRAY)
            else:
                depth_frame = None

        return color_frame, depth_frame
    
    def save_rgbd_triplet(self, color_image, depth_image):
        """Save RGB-D triplet (color, depth, metadata) in session folder"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # Include milliseconds
        
        files_saved = []
        
        # Save color image
        if color_image is not None:
            color_filename = f"color_{timestamp}.jpg"
            color_path = os.path.join(self.session_dir, color_filename)
            cv2.imwrite(color_path, color_image)
            files_saved.append("color")
        else:
            color_filename = None
        
        # Save depth image 
        if depth_image is not None:
            depth_filename = f"depth_{timestamp}.png"
            depth_path = os.path.join(self.session_dir, depth_filename)
            cv2.imwrite(depth_path, depth_image)
            files_saved.append("depth")
        else:
            depth_filename = None
        
        if not files_saved:
            print(" No frames to save!")
            return None
        
        # Create metadata JSON
        metadata = {
            "timestamp": timestamp,
            "session_dir": self.session_dir,
            "color_image": color_filename,
            "depth_image": depth_filename,
            "camera_model": "RealSense_ZR300_OpenCV",
            "color_format": "BGR",
            "depth_format": "8-bit/16-bit PNG",
            "color_device": self.color_device,
            "depth_device": self.depth_device,
            "files_saved": files_saved,
            "notes": f"RGB-D capture from ZR300 using OpenCV (devices: color={self.color_device}, depth={self.depth_device})"
        }
        
        json_filename = f"rgbd_{timestamp}.json"
        json_path = os.path.join(self.session_dir, json_filename)
        
        with open(json_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        print(f"Saved {'+'.join(files_saved)}: {timestamp}")
        return {
            'color': color_path if color_filename else None,
            'depth': depth_path if depth_filename else None,
            'metadata': json_path
        }
    
    def create_session_summary(self, captured_count):
        """Create session summary"""
        summary = {
            "session_timestamp": datetime.now().strftime("%Y%m%d_%H%M%S"),
            "session_directory": self.session_dir,
            "camera_model": "RealSense_ZR300_OpenCV",
            "color_device": self.color_device,
            "depth_device": self.depth_device,
            "total_captures": captured_count,
            "capture_format": "RGB-D triplets (color.jpg + depth.png + metadata.json)",
            "ready_for_conversion": True,
            "next_steps": [
                f"Run: python turn_RGBD_to_pointcloud.py {self.session_dir}",
                f"View: meshlab {self.session_dir}/pointclouds/*.ply"
            ]
        }
        summary_path = os.path.join(self.session_dir, "session_summary.json")
        with open(summary_path, 'w') as f:
            json.dump(summary, f, indent=2)
        print(f"Session summary: {summary_path}")
    
    def run_capture_session(self):
        """Main ZR300 capture loop"""
        print("\nInitializing RealSense ZR300...")
        if not self.setup_zr300_streams():
            print("Failed to initialize ZR300 streams!")
            return
        print(f"\nZR300 RGB-D Capture Session Started")
        print(f"Session: {os.path.basename(self.session_dir)}")
        print(f"Streams: Color={self.color_device}, Depth={self.depth_device}")
        print("\nControls:")
        print("  's' - Save RGB-D triplet")
        print("  'c' - Color stream only")
        print("  'd' - Depth stream only") 
        print("  'b' - Both streams")
        print("  'q' - Quit session")
        capture_count = 0
        display_mode = 'both'
        while True:
            color_image, depth_image = self.get_zr300_frames()
            # Display based on mode
            if display_mode == 'color' and color_image is not None:
                cv2.imshow(f'ZR300 Color Stream /dev/video{self.color_device}', color_image)
            elif display_mode == 'depth' and depth_image is not None:
                depth_colored = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.3), 
                    cv2.COLORMAP_JET
                )
                cv2.imshow(f'ZR300 Depth Stream /dev/video{self.depth_device}', depth_colored)
            elif display_mode == 'both':
                if color_image is not None:
                    cv2.imshow(f'ZR300 Color Stream /dev/video{self.color_device}', color_image)
                if depth_image is not None:
                    depth_colored = cv2.applyColorMap(
                        cv2.convertScaleAbs(depth_image, alpha=0.3), 
                        cv2.COLORMAP_JET
                    )
                    cv2.imshow(f'ZR300 Depth Stream /dev/video{self.depth_device}', depth_colored)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                result = self.save_rgbd_triplet(color_image, depth_image)
                if result:
                    capture_count += 1
            elif key == ord('c'):
                display_mode = 'color'
                cv2.destroyAllWindows()
            elif key == ord('d'):
                display_mode = 'depth'
                cv2.destroyAllWindows()
            elif key == ord('b'):
                display_mode = 'both'
                cv2.destroyAllWindows()
        # Finalize session
        self.create_session_summary(capture_count)
        print(f"\nZR300 Session Completed!")
        print(f"Captured: {capture_count} RGB-D triplets")
        print(f"Location: {self.session_dir}")
        if capture_count > 0:
            print(f"\nNext Steps:")
            print(f"1. Convert: python turn_RGBD_to_pointcloud.py")
            print(f"2. View: meshlab {self.session_dir}/pointclouds/*.ply")
        self.cleanup()
    
    def cleanup(self):
        """Clean up camera resources"""
        if self.color_cap:
            self.color_cap.release()
        if self.depth_cap:
            self.depth_cap.release()
        cv2.destroyAllWindows()
        print("Resources cleaned up")

def main():
    print("=== REALSENSE ZR300 RGB-D CAPTURE ===")
    print("Native OpenCV capture for ZR300 compatibility")
    print("No RealSense SDK dependency required!")
    print()
    session = ZR300RGBDCaptureSession()
    session.run_capture_session()

if __name__ == "__main__":
    main()