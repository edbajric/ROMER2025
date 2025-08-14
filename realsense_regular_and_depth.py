import cv2
import numpy as np
import os
from datetime import datetime

class RealSenseR200:
    """Specialized class for RealSense R200 camera"""
    
    def __init__(self):
        self.color_cap = None
        self.depth_cap = None
        self.find_streams()
    
    def find_streams(self):
        """Find color and depth streams"""
        print("Searching for R200 streams...")
        
        # Try to find color stream (YUYV format)
        for i in range(6):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                # Set to YUYV format for color
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','U','Y','V'))
                ret, frame = cap.read()
                if ret and len(frame.shape) == 3:  # Color image
                    print(f"✓ Color stream found on /dev/video{i}")
                    self.color_cap = cap
                    self.color_device = i
                    break
                cap.release()
        
        # Try to find depth stream (Y16 format)
        for i in range(6):
            if i == getattr(self, 'color_device', -1):
                continue  # Skip color device
            
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                # Try to set Y16 format for depth
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('Y','1','6',' '))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                ret, frame = cap.read()
                if ret:
                    print(f"✓ Depth stream found on /dev/video{i}")
                    self.depth_cap = cap
                    self.depth_device = i
                    break
                cap.release()
    
    def get_frames(self):
        """Get both color and depth frames"""
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
    
    def release(self):
        """Release camera resources"""
        if self.color_cap:
            self.color_cap.release()
        if self.depth_cap:
            self.depth_cap.release()

def main():
    """Main R200 capture application"""
    
    # Initialize R200
    r200 = RealSenseR200()
    
    if not r200.color_cap and not r200.depth_cap:
        print("No R200 streams found!")
        return
    
    print("\\nR200 Camera Controls:")
    print("  's' - Save color image")
    print("  'd' - Save depth image")
    print("  'b' - Save both (RGB-D pair)")
    print("  'v' - Start/stop video recording")
    print("  'q' - Quit")
    
    # Recording variables
    recording = False
    color_writer = None
    depth_writer = None
    
    while True:
        color_frame, depth_frame = r200.get_frames()
        
        # Display color frame
        if color_frame is not None:
            cv2.imshow('R200 Color', color_frame)
        
        # Display depth frame (if available)
        if depth_frame is not None:
            # Convert depth to displayable format
            if len(depth_frame.shape) == 2:  # Grayscale depth
                depth_display = cv2.applyColorMap(
                    cv2.convertScaleAbs(depth_frame, alpha=0.1), 
                    cv2.COLORMAP_JET
                )
            else:
                depth_display = depth_frame
            
            cv2.imshow('R200 Depth', depth_display)
        
        # Handle recording
        if recording:
            if color_frame is not None and color_writer:
                color_writer.write(color_frame)
            if depth_frame is not None and depth_writer:
                if len(depth_frame.shape) == 3:
                    depth_writer.write(depth_frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('s') and color_frame is not None:
            # Save color image
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            if not os.path.exists('r200_images'):
                os.makedirs('r200_images')
            
            filename = f"r200_images/color_{timestamp}.jpg"
            cv2.imwrite(filename, color_frame)
            print(f"Color image saved: {filename}")
            
        elif key == ord('d') and depth_frame is not None:
            # Save depth image
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            if not os.path.exists('r200_depth'):
                os.makedirs('r200_depth')
            
            # Save as 16-bit PNG to preserve depth precision
            if len(depth_frame.shape) == 2:  # Grayscale
                filename = f"r200_depth/depth_{timestamp}.png"
                cv2.imwrite(filename, depth_frame)
            else:
                filename = f"r200_depth/depth_{timestamp}.jpg"
                cv2.imwrite(filename, depth_frame)
            
            print(f"Depth image saved: {filename}")
            
        elif key == ord('b'):
            # Save RGB-D pair
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            saved_files = []
            
            if color_frame is not None:
                if not os.path.exists('r200_rgbd'):
                    os.makedirs('r200_rgbd')
                color_file = f"r200_rgbd/color_{timestamp}.jpg"
                cv2.imwrite(color_file, color_frame)
                saved_files.append(color_file)
            
            if depth_frame is not None:
                if not os.path.exists('r200_rgbd'):
                    os.makedirs('r200_rgbd')
                depth_file = f"r200_rgbd/depth_{timestamp}.png"
                cv2.imwrite(depth_file, depth_frame)
                saved_files.append(depth_file)
            
            if saved_files:
                print(f"RGB-D pair saved: {', '.join(saved_files)}")
            
        elif key == ord('v'):
            if not recording:
                # Start recording
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                if not os.path.exists('r200_videos'):
                    os.makedirs('r200_videos')
                
                if color_frame is not None:
                    h, w = color_frame.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    color_writer = cv2.VideoWriter(
                        f"r200_videos/color_{timestamp}.mp4", 
                        fourcc, 30.0, (w, h)
                    )
                
                if depth_frame is not None and len(depth_frame.shape) == 3:
                    h, w = depth_frame.shape[:2]
                    fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                    depth_writer = cv2.VideoWriter(
                        f"r200_videos/depth_{timestamp}.mp4", 
                        fourcc, 30.0, (w, h)
                    )
                
                recording = True
                print(f"Started recording RGB-D video: {timestamp}")
            else:
                # Stop recording
                recording = False
                if color_writer:
                    color_writer.release()
                    color_writer = None
                if depth_writer:
                    depth_writer.release()
                    depth_writer = None
                print("Recording stopped")
    
    # Cleanup
    r200.release()
    cv2.destroyAllWindows()
    
    if color_writer:
        color_writer.release()
    if depth_writer:
        depth_writer.release()

if __name__ == "__main__":
    main()
