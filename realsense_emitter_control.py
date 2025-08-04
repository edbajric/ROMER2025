import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

def realsense_ir_control():
    """Complete RealSense IR emitter control using proper SDK"""
    
    # Create a context
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        print("No RealSense devices found!")
        return
    
    # Get the first device
    device = devices[0]
    print(f"Found device: {device.get_info(rs.camera_info.name)}")
    
    # Create pipeline
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Configure streams
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    # Start streaming
    try:
        profile = pipeline.start(config)
        
        # Get depth sensor for IR emitter control
        depth_sensor = profile.get_device().first_depth_sensor()
        
        # Check if IR emitter is available
        if depth_sensor.supports(rs.option.emitter_enabled):
            print("✓ IR Emitter control available!")
            emitter_enabled = depth_sensor.get_option(rs.option.emitter_enabled)
            print(f"Current IR Emitter state: {'ON' if emitter_enabled else 'OFF'}")
        else:
            print("✗ IR Emitter control not available on this device")
            depth_sensor = None
        
        # Check laser power control
        laser_power_supported = False
        if depth_sensor and depth_sensor.supports(rs.option.laser_power):
            laser_power_supported = True
            current_power = depth_sensor.get_option(rs.option.laser_power)
            print(f"Current laser power: {current_power}")
        
        print("\\n=== REALSENSE IR EMITTER CONTROL ===")
        print("Controls:")
        print("  '1' - Turn IR emitter OFF")
        print("  '2' - Turn IR emitter ON")
        if laser_power_supported:
            print("  '3' - Decrease laser power")
            print("  '4' - Increase laser power")
        print("  'c' - Show color stream only")
        print("  'd' - Show depth stream")
        print("  'b' - Show both streams")
        print("  's' - Save current frames")
        print("  'q' - Quit")
        
        display_mode = 'both'  # 'color', 'depth', 'both'
        
        while True:
            # Get frames
            frames = pipeline.wait_for_frames()
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                continue
            
            # Convert to numpy arrays
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # Create colorized depth image
            colorizer = rs.colorizer()
            depth_colorized = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            
            # Add status text to color image
            if depth_sensor and depth_sensor.supports(rs.option.emitter_enabled):
                emitter_state = depth_sensor.get_option(rs.option.emitter_enabled)
                status = f"IR Emitter: {'ON' if emitter_state else 'OFF'}"
                
                if laser_power_supported:
                    power = depth_sensor.get_option(rs.option.laser_power)
                    status += f" | Power: {power:.0f}"
                
                cv2.putText(color_image, status, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Display based on mode
            if display_mode == 'color':
                cv2.imshow('RealSense Color (IR Control)', color_image)
            elif display_mode == 'depth':
                cv2.imshow('RealSense Depth', depth_colorized)
            else:  # both
                # Stack images horizontally
                combined = np.hstack((color_image, depth_colorized))
                cv2.imshow('RealSense Color + Depth', combined)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('1') and depth_sensor:
                # Turn OFF IR emitter
                if depth_sensor.supports(rs.option.emitter_enabled):
                    depth_sensor.set_option(rs.option.emitter_enabled, 0)
                    print("IR Emitter turned OFF")
                    print("Note: This will disable depth sensing!")
                
            elif key == ord('2') and depth_sensor:
                # Turn ON IR emitter
                if depth_sensor.supports(rs.option.emitter_enabled):
                    depth_sensor.set_option(rs.option.emitter_enabled, 1)
                    print("IR Emitter turned ON")
                
            elif key == ord('3') and depth_sensor and laser_power_supported:
                # Decrease laser power
                current_power = depth_sensor.get_option(rs.option.laser_power)
                new_power = max(current_power - 30, 0)
                depth_sensor.set_option(rs.option.laser_power, new_power)
                print(f"Laser power decreased to {new_power}")
                
            elif key == ord('4') and depth_sensor and laser_power_supported:
                # Increase laser power
                current_power = depth_sensor.get_option(rs.option.laser_power)
                power_range = depth_sensor.get_option_range(rs.option.laser_power)
                new_power = min(current_power + 30, power_range.max)
                depth_sensor.set_option(rs.option.laser_power, new_power)
                print(f"Laser power increased to {new_power}")
                
            elif key == ord('c'):
                display_mode = 'color'
                cv2.destroyAllWindows()
                print("Showing color stream only")
                
            elif key == ord('d'):
                display_mode = 'depth'
                cv2.destroyAllWindows()
                print("Showing depth stream only")
                
            elif key == ord('b'):
                display_mode = 'both'
                cv2.destroyAllWindows()
                print("Showing both streams")
                
            elif key == ord('s'):
                # Save frames
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                if not os.path.exists('realsense_captures'):
                    os.makedirs('realsense_captures')
                
                # Save color image
                cv2.imwrite(f"realsense_captures/color_{timestamp}.jpg", color_image)
                
                # Save depth as PNG (preserves 16-bit data)
                cv2.imwrite(f"realsense_captures/depth_{timestamp}.png", depth_image)
                
                # Save colorized depth
                cv2.imwrite(f"realsense_captures/depth_color_{timestamp}.jpg", depth_colorized)
                
                print(f"Frames saved with timestamp {timestamp}")
    
    except Exception as e:
        print(f"Error: {e}")
    
    finally:
        pipeline.stop()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    print("=== REALSENSE SDK IR EMITTER CONTROL ===")
    print()
    print("This script uses the proper RealSense SDK for full IR control.")
    print("You can completely turn off the purple laser dots!")
    print()
    input("Press Enter to start...")
    
    realsense_ir_control()
