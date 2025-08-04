# doesnt work with R200 but does with D455

import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

def detect_realsense_model():
    """Detect which RealSense camera model is connected"""
    ctx = rs.context()
    devices = ctx.query_devices()
    
    if len(devices) == 0:
        return None, None
    
    device = devices[0]
    name = device.get_info(rs.camera_info.name)
    
    # Check for different models
    if "R200" in name:
        return device, "R200"
    elif "D435" in name or "D455" in name or "D415" in name:
        return device, "D4xx"
    elif "SR300" in name:
        return device, "SR300"
    else:
        return device, "Unknown"

def realsense_r200_control():
    """RealSense R200 specific IR emitter control"""
    
    device, model = detect_realsense_model()
    if not device:
        print("No RealSense devices found!")
        return
    
    print(f"Detected: {device.get_info(rs.camera_info.name)} ({model})")
    
    # R200 specific configuration
    pipeline = rs.pipeline()
    config = rs.config()
    
    try:
        # R200 typically supports these streams
        if model == "R200":
            # R200 stream configuration
            config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
            config.enable_stream(rs.stream.depth, 628, 468, rs.format.z16, 30)  # R200 specific resolution
        else:
            # Modern RealSense configuration
            config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
            config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
        
        profile = pipeline.start(config)
        
        # Get available sensors
        sensors = device.query_sensors()
        depth_sensor = None
        color_sensor = None
        
        print("\\nAvailable sensors:")
        for i, sensor in enumerate(sensors):
            sensor_name = sensor.get_info(rs.camera_info.name) if sensor.supports(rs.camera_info.name) else f"Sensor {i}"
            print(f"  {i}: {sensor_name}")
            
            # Find depth sensor (usually contains "Depth" or "Stereo")
            if "Depth" in sensor_name or "Stereo" in sensor_name:
                depth_sensor = sensor
            elif "RGB" in sensor_name or "Color" in sensor_name:
                color_sensor = sensor
        
        # Check IR emitter control options
        ir_control_available = False
        laser_power_available = False
        
        if depth_sensor:
            print(f"\\nDepth sensor options:")
            
            # Check all available options for R200
            options_to_check = [
                (rs.option.emitter_enabled, "IR Emitter"),
                (rs.option.laser_power, "Laser Power"),
                (rs.option.projector_temperature, "Projector Temperature"),
                (rs.option.emitter_on_off, "Emitter On/Off"),
                (rs.option.enable_auto_exposure, "Auto Exposure"),
                (rs.option.exposure, "Exposure"),
                (rs.option.gain, "Gain"),
            ]
            
            for option, name in options_to_check:
                if depth_sensor.supports(option):
                    try:
                        current_value = depth_sensor.get_option(option)
                        option_range = depth_sensor.get_option_range(option)
                        print(f"  ✓ {name}: {current_value} (range: {option_range.min}-{option_range.max})")
                        
                        if option == rs.option.emitter_enabled or option == rs.option.emitter_on_off:
                            ir_control_available = True
                        elif option == rs.option.laser_power:
                            laser_power_available = True
                            
                    except Exception as e:
                        print(f"  ✗ {name}: Error reading - {e}")
                else:
                    print(f"  ✗ {name}: Not supported")
        
        # Alternative: Try to control via different methods for R200
        if not ir_control_available and model == "R200":
            print("\\nTrying R200-specific IR control methods...")
            
            # R200 sometimes uses different option names
            alt_options = [
                rs.option.emitter_on_off,
                rs.option.projector_temperature,
            ]
            
            for option in alt_options:
                if depth_sensor and depth_sensor.supports(option):
                    print(f"Found alternative control: {option}")
                    ir_control_available = True
                    break
        
        print("\\n=== REALSENSE IR CONTROL ===")
        print(f"Model: {model}")
        print(f"IR Control: {'Available' if ir_control_available else 'Limited'}")
        print(f"Laser Power: {'Available' if laser_power_available else 'Not Available'}")
        
        print("\\nControls:")
        if ir_control_available:
            print("  '1' - Turn IR emitter OFF")
            print("  '2' - Turn IR emitter ON")
        if laser_power_available:
            print("  '3' - Decrease laser power")
            print("  '4' - Increase laser power")
        print("  '5' - Adjust exposure (reduce IR visibility)")
        print("  '6' - Reset exposure")
        print("  'c' - Color stream only")
        print("  'd' - Depth stream only")
        print("  'b' - Both streams")
        print("  's' - Save frames")
        print("  'q' - Quit")
        
        display_mode = 'both'
        
        while True:
            try:
                frames = pipeline.wait_for_frames()
                color_frame = frames.get_color_frame()
                depth_frame = frames.get_depth_frame()
                
                if not color_frame:
                    print("No color frame")
                    continue
                
                # Convert frames
                color_image = np.asanyarray(color_frame.get_data())
                
                # Handle color format differences
                if model == "R200" and color_image.shape[2] == 3:
                    # R200 might give RGB, convert to BGR for OpenCV
                    color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
                
                depth_colorized = None
                if depth_frame:
                    depth_image = np.asanyarray(depth_frame.get_data())
                    colorizer = rs.colorizer()
                    depth_colorized = np.asanyarray(colorizer.colorize(depth_frame).get_data())
                
                # Add status overlay
                status_lines = [f"Model: {model}"]
                
                if depth_sensor and ir_control_available:
                    try:
                        if depth_sensor.supports(rs.option.emitter_enabled):
                            emitter_state = depth_sensor.get_option(rs.option.emitter_enabled)
                            status_lines.append(f"IR: {'ON' if emitter_state else 'OFF'}")
                        elif depth_sensor.supports(rs.option.emitter_on_off):
                            emitter_state = depth_sensor.get_option(rs.option.emitter_on_off)
                            status_lines.append(f"IR: {'ON' if emitter_state else 'OFF'}")
                    except:
                        status_lines.append("IR: Unknown")
                
                if depth_sensor and laser_power_available:
                    try:
                        power = depth_sensor.get_option(rs.option.laser_power)
                        status_lines.append(f"Power: {power:.0f}")
                    except:
                        pass
                
                # Draw status
                for i, line in enumerate(status_lines):
                    cv2.putText(color_image, line, (10, 30 + i*25), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Display
                if display_mode == 'color':
                    cv2.imshow('RealSense Color', color_image)
                elif display_mode == 'depth' and depth_colorized is not None:
                    cv2.imshow('RealSense Depth', depth_colorized)
                elif display_mode == 'both' and depth_colorized is not None:
                    # Resize if needed
                    if color_image.shape[:2] != depth_colorized.shape[:2]:
                        depth_colorized = cv2.resize(depth_colorized, (color_image.shape[1], color_image.shape[0]))
                    combined = np.hstack((color_image, depth_colorized))
                    cv2.imshow('RealSense Color + Depth', combined)
                else:
                    cv2.imshow('RealSense Color', color_image)
                
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord('1') and ir_control_available and depth_sensor:
                    # Turn OFF IR emitter
                    try:
                        if depth_sensor.supports(rs.option.emitter_enabled):
                            depth_sensor.set_option(rs.option.emitter_enabled, 0)
                            print("IR Emitter turned OFF")
                        elif depth_sensor.supports(rs.option.emitter_on_off):
                            depth_sensor.set_option(rs.option.emitter_on_off, 0)
                            print("IR Emitter turned OFF (alternative method)")
                    except Exception as e:
                        print(f"Could not turn off IR: {e}")
                    
                elif key == ord('2') and ir_control_available and depth_sensor:
                    # Turn ON IR emitter
                    try:
                        if depth_sensor.supports(rs.option.emitter_enabled):
                            depth_sensor.set_option(rs.option.emitter_enabled, 1)
                            print("IR Emitter turned ON")
                        elif depth_sensor.supports(rs.option.emitter_on_off):
                            depth_sensor.set_option(rs.option.emitter_on_off, 1)
                            print("IR Emitter turned ON (alternative method)")
                    except Exception as e:
                        print(f"Could not turn on IR: {e}")
                    
                elif key == ord('3') and laser_power_available and depth_sensor:
                    # Decrease laser power
                    try:
                        current_power = depth_sensor.get_option(rs.option.laser_power)
                        new_power = max(current_power - 30, 0)
                        depth_sensor.set_option(rs.option.laser_power, new_power)
                        print(f"Laser power: {new_power}")
                    except Exception as e:
                        print(f"Could not adjust laser power: {e}")
                    
                elif key == ord('4') and laser_power_available and depth_sensor:
                    # Increase laser power
                    try:
                        current_power = depth_sensor.get_option(rs.option.laser_power)
                        power_range = depth_sensor.get_option_range(rs.option.laser_power)
                        new_power = min(current_power + 30, power_range.max)
                        depth_sensor.set_option(rs.option.laser_power, new_power)
                        print(f"Laser power: {new_power}")
                    except Exception as e:
                        print(f"Could not adjust laser power: {e}")
                
                elif key == ord('5'):
                    # Adjust exposure to reduce IR visibility
                    if color_sensor and color_sensor.supports(rs.option.exposure):
                        try:
                            current_exp = color_sensor.get_option(rs.option.exposure)
                            new_exp = max(current_exp * 0.8, 1)
                            color_sensor.set_option(rs.option.exposure, new_exp)
                            print(f"Exposure reduced to {new_exp}")
                        except Exception as e:
                            print(f"Could not adjust exposure: {e}")
                    
                elif key == ord('6'):
                    # Reset exposure
                    if color_sensor and color_sensor.supports(rs.option.exposure):
                        try:
                            color_sensor.set_option(rs.option.enable_auto_exposure, 1)
                            print("Auto exposure enabled")
                        except Exception as e:
                            print(f"Could not reset exposure: {e}")
                
                elif key == ord('c'):
                    display_mode = 'color'
                    cv2.destroyAllWindows()
                elif key == ord('d'):
                    display_mode = 'depth'
                    cv2.destroyAllWindows()
                elif key == ord('b'):
                    display_mode = 'both'
                    cv2.destroyAllWindows()
                elif key == ord('s'):
                    # Save frames
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    if not os.path.exists('realsense_captures'):
                        os.makedirs('realsense_captures')
                    
                    cv2.imwrite(f"realsense_captures/color_{model}_{timestamp}.jpg", color_image)
                    if depth_frame:
                        cv2.imwrite(f"realsense_captures/depth_{model}_{timestamp}.png", depth_image)
                    print(f"Frames saved: {timestamp}")
                    
            except Exception as e:
                print(f"Frame processing error: {e}")
                break
    
    except Exception as e:
        print(f"Error: {e}")
        print("\\nTroubleshooting:")
        print("1. Make sure RealSense camera is connected")
        print("2. Try different USB port")
        print("3. Check if other software is using the camera")
    
    finally:
        try:
            pipeline.stop()
        except:
            pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    print("=== UNIVERSAL REALSENSE IR CONTROL ===")
    print("Supports R200, D435, D455, SR300 and other models")
    print()
    input("Press Enter to detect and control your RealSense camera...")
    
    realsense_r200_control()
