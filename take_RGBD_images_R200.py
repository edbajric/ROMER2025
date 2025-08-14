import cv2
import numpy as np
import os
import json
from datetime import datetime
import subprocess
import time

class R200RGBDCaptureSession:
    """R200-specific RGB-D capture using OpenCV and V4L2"""
    
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
        
    def detect_r200_devices(self):
        """Detect R200 video devices using v4l2"""
        print(" Scanning for RealSense R200 devices...")
        
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
                    
                    if "Intel" in result.stdout and "RealSense" in result.stdout:
                        # Get supported formats
                        formats_result = subprocess.run(
                            ['v4l2-ctl', '-d', device_path, '--list-formats-ext'], 
                            capture_output=True, text=True, timeout=3
                        )
                        
                        formats_output = formats_result.stdout
                        
                        # Determine if this is color or depth
                        if "YUYV" in formats_output or "RGB" in formats_output:
                            color_candidates.append(i)
                            print(f"  ✓ Color stream candidate: /dev/video{i}")
                        elif "Y16" in formats_output or "GREY" in formats_output:
                            depth_candidates.append(i)
                            print(f"  ✓ Depth stream candidate: /dev/video{i}")
                        else:
                            print(f"  ? Unknown stream: /dev/video{i}")
                            
                except (subprocess.TimeoutExpired, subprocess.CalledProcessError, FileNotFoundError):
                    # v4l2-ctl not available or device busy, try OpenCV probe
                    cap = cv2.VideoCapture(i)
                    if cap.isOpened():
                        ret, frame = cap.read()
                        if ret:
                            if len(frame.shape) == 3 and frame.shape[2] == 3:
                                color_candidates.append(i)
                                print(f"  ✓ Color stream (OpenCV): /dev/video{i}")
                            elif len(frame.shape) == 2 or (len(frame.shape) == 3 and np.mean(frame) < 50):
                                depth_candidates.append(i)
                                print(f"  ✓ Depth stream (OpenCV): /dev/video{i}")
                        cap.release()
        
        return color_candidates, depth_candidates
    
    def setup_r200_streams(self):
        """Setup R200 color and depth streams"""
        color_candidates, depth_candidates = self.detect_r200_devices()
        
        if not color_candidates and not depth_candidates:
            print(" No R200 devices detected!")
            print(" Make sure:")
            print("   - R200 is connected via USB 3.0")
            print("   - No other software is using the camera")
            print("   - Try: sudo chmod 666 /dev/video*")
            return False
        
        # Setup color stream
        if color_candidates:
            for device_id in color_candidates:
                try:
                    cap = cv2.VideoCapture(device_id)
                    if cap.isOpened():
                        # R200 color stream settings
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                        cap.set(cv2.CAP_PROP_FPS, 30)
                        
                        # Test capture
                        ret, frame = cap.read()
                        if ret and len(frame.shape) == 3:
                            self.color_cap = cap
                            self.color_device = device_id
                            print(f" Color stream ready: /dev/video{device_id}")
                            break
                        cap.release()
                except Exception as e:
                    print(f"  Failed to setup color on /dev/video{device_id}: {e}")

        # Setup depth stream
        if depth_candidates:
            for device_id in depth_candidates:
                if device_id == self.color_device:
                    continue  # Skip if already used for color
                    
                try:
                    cap = cv2.VideoCapture(device_id)
                    if cap.isOpened():
                        # R200 depth stream settings
                        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 628)  # R200 native depth resolution
                        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 468)
                        cap.set(cv2.CAP_PROP_FPS, 30)
                        
                        # Test capture
                        ret, frame = cap.read()
                        if ret:
                            self.depth_cap = cap
                            self.depth_device = device_id
                            print(f" Depth stream ready: /dev/video{device_id}")
                            break
                        cap.release()
                except Exception as e:
                    print(f"  Failed to setup depth on /dev/video{device_id}: {e}")
        
        success = self.color_cap is not None or self.depth_cap is not None
        
        if success:
            print(f"\n📷 R200 Setup Complete:")
            print(f"   Color: {'' if self.color_cap else ''} /dev/video{self.color_device if self.color_device else 'N/A'}")
            print(f"   Depth: {'' if self.depth_cap else ''} /dev/video{self.depth_device if self.depth_device else 'N/A'}")
        
        return success
    
    def get_r200_frames(self):
        """Get frames from R200"""
        color_frame = None
        depth_frame = None
        
        # Get color frame
        if self.color_cap:
            ret, color_frame = self.color_cap.read()
            if not ret:
                color_frame = None
        
        # Get depth frame
        if self.depth_cap:
            ret, depth_frame = self.depth_cap.read()
            if not ret:
                depth_frame = None
            else:
                # Process depth frame for R200
                if len(depth_frame.shape) == 3:
                    # Convert to grayscale if needed
                    depth_frame = cv2.cvtColor(depth_frame, cv2.COLOR_BGR2GRAY)
        
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
            "camera_model": "RealSense_R200_OpenCV",
            "color_format": "BGR",
            "depth_format": "8-bit/16-bit PNG",
            "color_device": self.color_device,
            "depth_device": self.depth_device,
            "files_saved": files_saved,
            "notes": f"RGB-D capture from R200 using OpenCV (devices: color={self.color_device}, depth={self.depth_device})"
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
            "camera_model": "RealSense_R200_OpenCV",
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
        
        print(f" Session summary: {summary_path}")
    
    def run_capture_session(self):
        """Main R200 capture loop"""
        print("\n Initializing RealSense R200...")
        
        if not self.setup_r200_streams():
            print(" Failed to initialize R200 streams!")
            return
        
        print(f"\n R200 RGB-D Capture Session Started")
        print(f" Session: {os.path.basename(self.session_dir)}")
        print(f" Streams: Color={self.color_device}, Depth={self.depth_device}")
        print("\n⌨  Controls:")
        print("   's' - Save RGB-D triplet")
        print("   'c' - Color stream only")
        print("   'd' - Depth stream only") 
        print("   'b' - Both streams")
        print("   'q' - Quit session")
        
        capture_count = 0
        display_mode = 'both'
        
        while True:
            color_image, depth_image = self.get_r200_frames()
            
            # Display based on mode
            if display_mode == 'color' and color_image is not None:
                display_frame = color_image.copy()
                
                # Add info overlay
                cv2.putText(display_frame, f"R200 Session: {os.path.basename(self.session_dir)}", 
                           (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(display_frame, f"Captures: {capture_count}", 
                           (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                cv2.putText(display_frame, f"Mode: Color Only", 
                           (10, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                
                cv2.imshow('R200 RGB-D Capture', display_frame)
                
            elif display_mode == 'depth' and depth_image is not None:
                # Convert depth to colorized display
                depth_colored = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_image, alpha=0.3), 
                    cv2.COLORMAP_JET
                )
                
                cv2.putText(depth_colored, f"R200 Depth: {capture_count} captures", 
                           (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                cv2.imshow('R200 RGB-D Capture', depth_colored)
                
            elif display_mode == 'both':
                frames_to_show = []
                
                if color_image is not None:
                    color_display = color_image.copy()
                    cv2.putText(color_display, "Color", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                    frames_to_show.append(color_display)
                
                if depth_image is not None:
                    depth_colored = cv2.applyColorMap(
                        cv2.convertScaleAbs(depth_image, alpha=0.3), 
                        cv2.COLORMAP_JET
                    )
                    cv2.putText(depth_colored, "Depth", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                    frames_to_show.append(depth_colored)
                
                if frames_to_show:
                    if len(frames_to_show) == 2:
                        # Side by side
                        combined = np.hstack(frames_to_show)
                    else:
                        combined = frames_to_show[0]
                    
                    cv2.putText(combined, f"R200 Session - Captures: {capture_count}", 
                               (10, combined.shape[0]-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)
                    
                    cv2.imshow('R200 RGB-D Capture', combined)
            
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
        print(f"\n R200 Session Completed!")
        print(f" Captured: {capture_count} RGB-D triplets")
        print(f" Location: {self.session_dir}")
        
        if capture_count > 0:
            print(f"\n🔄 Next Steps:")
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
        print("🧹 Resources cleaned up")

def main():
    print("=== REALSENSE R200 RGB-D CAPTURE ===")
    print("Native OpenCV capture for R200 compatibility")
    print("No RealSense SDK dependency required!")
    print()
    
    session = R200RGBDCaptureSession()
    session.run_capture_session()

if __name__ == "__main__":
    main()