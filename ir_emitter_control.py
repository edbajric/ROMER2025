# this one shows the purple dots from IR emitter
# only works with D455, not R200





import cv2
import numpy as np
import os
from datetime import datetime

def find_realsense_streams():
    """Find RealSense camera streams and their capabilities"""
    print("Scanning for RealSense streams...")
    
    streams = []
    for i in range(10):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                
                # Check if this looks like an IR stream (grayscale or weird colors)
                if len(frame.shape) == 2 or np.mean(frame) < 50:
                    stream_type = "IR/Depth"
                else:
                    stream_type = "Color"
                
                streams.append({
                    'device': i,
                    'resolution': f"{width}x{height}",
                    'type': stream_type,
                    'cap': cap
                })
                print(f"Device {i}: {width}x{height} - {stream_type}")
            else:
                cap.release()
    
    return streams

def create_ir_control_interface():
    """Create interface to control IR emitter and camera settings"""
    
    streams = find_realsense_streams()
    if not streams:
        print("No camera streams found!")
        return
    
    print(f"\\nFound {len(streams)} camera streams")
    print("\\nChoose a stream to control:")
    for i, stream in enumerate(streams):
        print(f"{i}: Device {stream['device']} - {stream['resolution']} ({stream['type']})")
    
    try:
        choice = int(input("\\nEnter stream number: "))
        selected_stream = streams[choice]
        cap = selected_stream['cap']
    except (ValueError, IndexError):
        print("Invalid choice")
        return
    
    # Close other streams
    for i, stream in enumerate(streams):
        if i != choice:
            stream['cap'].release()
    
    print(f"\\nSelected Device {selected_stream['device']}")
    print("\\nControls:")
    print("  '1' - Try to disable IR emitter")
    print("  '2' - Try to enable IR emitter") 
    print("  '3' - Reduce IR sensitivity (lower exposure)")
    print("  '4' - Increase IR sensitivity (higher exposure)")
    print("  '5' - Auto exposure ON")
    print("  '6' - Auto exposure OFF")
    print("  'r' - Reset all settings")
    print("  's' - Save current frame")
    print("  'q' - Quit")
    
    # Try to get current settings
    current_exposure = cap.get(cv2.CAP_PROP_EXPOSURE)
    current_gain = cap.get(cv2.CAP_PROP_GAIN)
    current_brightness = cap.get(cv2.CAP_PROP_BRIGHTNESS)
    
    print(f"\\nCurrent Settings:")
    print(f"  Exposure: {current_exposure}")
    print(f"  Gain: {current_gain}")
    print(f"  Brightness: {current_brightness}")
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to read frame")
            break
        
        # Display current settings on frame
        settings_text = f"Exp: {cap.get(cv2.CAP_PROP_EXPOSURE):.1f} | Gain: {cap.get(cv2.CAP_PROP_GAIN):.1f}"
        cv2.putText(frame, settings_text, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        # Check if frame has purple dots (high magenta content)
        if len(frame.shape) == 3:
            # Convert to HSV to detect magenta/purple
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
            purple_mask = cv2.inRange(hsv, (120, 50, 50), (160, 255, 255))
            purple_pixels = cv2.countNonZero(purple_mask)
            
            ir_status = "IR DOTS DETECTED" if purple_pixels > 1000 else "Clean image"
            color = (0, 0, 255) if purple_pixels > 1000 else (0, 255, 0)
            cv2.putText(frame, ir_status, (10, 60), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2)
        
        cv2.imshow(f'RealSense Control - Device {selected_stream["device"]}', frame)
        
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q'):
            break
        elif key == ord('1'):
            # Try to disable IR emitter (this might not work with generic OpenCV)
            print("Attempting to disable IR emitter...")
            cap.set(cv2.CAP_PROP_EXPOSURE, -8)  # Very low exposure
            cap.set(cv2.CAP_PROP_GAIN, 0)       # Low gain
            print("IR emitter control attempted (may require RealSense SDK)")
            
        elif key == ord('2'):
            # Try to enable IR emitter
            print("Attempting to enable IR emitter...")
            cap.set(cv2.CAP_PROP_EXPOSURE, -4)  # Higher exposure
            cap.set(cv2.CAP_PROP_GAIN, 50)      # Higher gain
            
        elif key == ord('3'):
            # Reduce IR sensitivity
            current_exp = cap.get(cv2.CAP_PROP_EXPOSURE)
            new_exp = max(current_exp - 1, -10)
            cap.set(cv2.CAP_PROP_EXPOSURE, new_exp)
            print(f"Reduced exposure to {new_exp}")
            
        elif key == ord('4'):
            # Increase IR sensitivity
            current_exp = cap.get(cv2.CAP_PROP_EXPOSURE)
            new_exp = min(current_exp + 1, 0)
            cap.set(cv2.CAP_PROP_EXPOSURE, new_exp)
            print(f"Increased exposure to {new_exp}")
            
        elif key == ord('5'):
            # Auto exposure ON
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
            print("Auto exposure enabled")
            
        elif key == ord('6'):
            # Auto exposure OFF
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25)
            print("Auto exposure disabled")
            
        elif key == ord('r'):
            # Reset settings
            cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.75)
            cap.set(cv2.CAP_PROP_EXPOSURE, -4)
            cap.set(cv2.CAP_PROP_GAIN, 30)
            cap.set(cv2.CAP_PROP_BRIGHTNESS, 0)
            print("Settings reset to defaults")
            
        elif key == ord('s'):
            # Save frame
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            if not os.path.exists('ir_control_images'):
                os.makedirs('ir_control_images')
            filename = f"ir_control_images/frame_{timestamp}.jpg"
            cv2.imwrite(filename, frame)
            print(f"Frame saved: {filename}")
    
    cap.release()
    cv2.destroyAllWindows()

def check_realsense_sdk():
    """Check if RealSense SDK is available for better control"""
    try:
        import pyrealsense2 as rs
        print("✓ RealSense SDK (pyrealsense2) is available!")
        print("  This allows proper IR emitter control")
        return True
    except ImportError:
        print("✗ RealSense SDK not available")
        print("  Install with: pip install pyrealsense2")
        print("  For full IR emitter control")
        return False

if __name__ == "__main__":
    print("=== REALSENSE IR EMITTER CONTROL ===")
    print()
    
    # Check if proper SDK is available
    has_sdk = check_realsense_sdk()
    
    print()
    print("This tool helps you control the IR laser dots you're seeing.")
    print("The purple dots are infrared structured light for depth sensing.")
    print()
    
    if not has_sdk:
        print("⚠ Limited control with OpenCV only")
        print("  Install pyrealsense2 for full control")
    
    print()
    input("Press Enter to start camera control...")
    
    create_ir_control_interface()
