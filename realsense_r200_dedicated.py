# doesnt work with R200 but does with D455




import pyrealsense2 as rs
import numpy as np
import cv2
import os
from datetime import datetime

def list_all_devices():
    """List all available devices with detailed info"""
    print("=== DEVICE DETECTION ===")
    
    ctx = rs.context()
    devices = ctx.query_devices()
    
    print(f"Found {len(devices)} RealSense devices:")
    
    for i, device in enumerate(devices):
        print(f"\nDevice {i}:")
        try:
            name = device.get_info(rs.camera_info.name) if device.supports(rs.camera_info.name) else "Unknown"
            serial = device.get_info(rs.camera_info.serial_number) if device.supports(rs.camera_info.serial_number) else "Unknown"
            firmware = device.get_info(rs.camera_info.firmware_version) if device.supports(rs.camera_info.firmware_version) else "Unknown"
            product_id = device.get_info(rs.camera_info.product_id) if device.supports(rs.camera_info.product_id) else "Unknown"
            
            print(f"  Name: {name}")
            print(f"  Serial: {serial}")
            print(f"  Firmware: {firmware}")
            print(f"  Product ID: {product_id}")
            
            # List available sensors
            sensors = device.query_sensors()
            print(f"  Sensors ({len(sensors)}): ")
            for j, sensor in enumerate(sensors):
                sensor_name = sensor.get_info(rs.camera_info.name) if sensor.supports(rs.camera_info.name) else f"Sensor {j}"
                print(f"    {j}: {sensor_name}")
                
        except Exception as e:
            print(f"  Error reading device info: {e}")
    
    return devices

def try_r200_connection():
    """Try multiple methods to connect to R200"""
    print("\n=== R200 CONNECTION ATTEMPTS ===")
    
    devices = list_all_devices()
    
    if len(devices) == 0:
        print("\nNo devices found. Trying alternative detection methods...")
        
        # Try OpenCV camera detection
        print("\nTrying OpenCV camera detection:")
        for i in range(10):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    print(f"  Camera {i}: Working ({frame.shape})")
                cap.release()
            else:
                print(f"  Camera {i}: Not available")
        
        return None
    
    # Try each device
    for i, device in enumerate(devices):
        print(f"\n--- Trying Device {i} ---")
        
        try:
            name = device.get_info(rs.camera_info.name) if device.supports(rs.camera_info.name) else "Unknown"
            print(f"Device name: {name}")
            
            # R200 specific attempt
            pipeline = rs.pipeline()
            config = rs.config()
            
            # Try different R200 configurations
            r200_configs = [
                # Config 1: Standard R200 resolution
                {
                    'color': (640, 480, rs.format.bgr8, 30),
                    'depth': (628, 468, rs.format.z16, 30)
                },
                # Config 2: Alternative R200 resolution
                {
                    'color': (640, 480, rs.format.rgb8, 30),
                    'depth': (628, 468, rs.format.z16, 30)
                },
                # Config 3: Lower resolution
                {
                    'color': (320, 240, rs.format.bgr8, 30),
                    'depth': (320, 240, rs.format.z16, 30)
                },
                # Config 4: Color only
                {
                    'color': (640, 480, rs.format.bgr8, 30),
                    'depth': None
                }
            ]
            
            for config_idx, cfg in enumerate(r200_configs):
                print(f"  Trying config {config_idx + 1}...")
                
                try:
                    config = rs.config()
                    config.enable_device(device.get_info(rs.camera_info.serial_number))
                    
                    # Enable streams
                    color_cfg = cfg['color']
                    config.enable_stream(rs.stream.color, color_cfg[0], color_cfg[1], color_cfg[2], color_cfg[3])
                    
                    if cfg['depth']:
                        depth_cfg = cfg['depth']
                        config.enable_stream(rs.stream.depth, depth_cfg[0], depth_cfg[1], depth_cfg[2], depth_cfg[3])
                    
                    # Try to start
                    profile = pipeline.start(config)
                    print(f"  ✓ Config {config_idx + 1} WORKS!")
                    
                    # Test frame capture
                    frames = pipeline.wait_for_frames(timeout_ms=5000)
                    color_frame = frames.get_color_frame()
                    
                    if color_frame:
                        print(f"  ✓ Color frame captured: {color_frame.get_width()}x{color_frame.get_height()}")
                        return pipeline, device, profile
                    else:
                        print(f"  ✗ No color frame received")
                        pipeline.stop()
                        
                except Exception as e:
                    print(f"  ✗ Config {config_idx + 1} failed: {e}")
                    try:
                        pipeline.stop()
                    except:
                        pass
                    pipeline = rs.pipeline()  # Create new pipeline for next attempt
            
        except Exception as e:
            print(f"Device {i} connection failed: {e}")
    
    return None

def r200_ir_control():
    """R200 specific IR emitter control with multiple fallback methods"""
    
    connection_result = try_r200_connection()
    
    if not connection_result:
        print("\n❌ Could not establish connection to R200 camera")
        print("\nTroubleshooting steps:")
        print("1. Make sure R200 is connected via USB 3.0")
        print("2. Try different USB port")
        print("3. Restart the camera (unplug/replug)")
        print("4. Check if other software is using the camera")
        print("5. Update RealSense SDK drivers")
        print("6. Try running as administrator/sudo")
        return
    
    pipeline, device, profile = connection_result
    
    print("\n✅ R200 CONNECTION SUCCESSFUL!")
    
    # Get sensors for IR control
    sensors = device.query_sensors()
    depth_sensor = None
    color_sensor = None
    
    print("\nAnalyzing sensors for IR control...")
    for i, sensor in enumerate(sensors):
        sensor_name = sensor.get_info(rs.camera_info.name) if sensor.supports(rs.camera_info.name) else f"Sensor {i}"
        print(f"  Sensor {i}: {sensor_name}")
        
        # R200 sensor identification
        if "Depth" in sensor_name or "IR" in sensor_name or "Stereo" in sensor_name:
            depth_sensor = sensor
            print(f"    → Using as depth/IR sensor")
        elif "RGB" in sensor_name or "Color" in sensor_name:
            color_sensor = sensor
            print(f"    → Using as color sensor")
    
    # Check IR control options for R200
    ir_options = []
    if depth_sensor:
        print(f"\nChecking IR control options on depth sensor...")
        
        # R200 specific options to check
        r200_options = [
            (rs.option.emitter_enabled, "IR Emitter Enabled"),
            (rs.option.emitter_on_off, "IR Emitter On/Off"),
            (rs.option.laser_power, "Laser Power"),
            (rs.option.projector_temperature, "Projector Temperature"),
            (rs.option.enable_auto_exposure, "Auto Exposure"),
            (rs.option.exposure, "Exposure"),
            (rs.option.gain, "Gain"),
        ]
        
        for option, name in r200_options:
            if depth_sensor.supports(option):
                try:
                    current_value = depth_sensor.get_option(option)
                    option_range = depth_sensor.get_option_range(option)
                    print(f"  ✓ {name}: {current_value} (range: {option_range.min}-{option_range.max})")
                    ir_options.append((option, name))
                except Exception as e:
                    print(f"  ⚠ {name}: Supported but error reading - {e}")
            else:
                print(f"  ✗ {name}: Not supported")
    
    if not ir_options:
        print("⚠ No IR control options found - R200 may have limited control")
    
    print("\n=== R200 IR CONTROL INTERFACE ===")
    print("Controls:")
    if any(opt[0] in [rs.option.emitter_enabled, rs.option.emitter_on_off] for opt in ir_options):
        print("  '1' - Turn IR emitter OFF")
        print("  '2' - Turn IR emitter ON")
    if any(opt[0] == rs.option.laser_power for opt in ir_options):
        print("  '3' - Decrease laser power")
        print("  '4' - Increase laser power")
    print("  '5' - Reduce exposure (minimize IR visibility)")
    print("  '6' - Reset to auto exposure")
    print("  's' - Save current frame")
    print("  'q' - Quit")
    
    try:
        while True:
            try:
                frames = pipeline.wait_for_frames(timeout_ms=1000)
                color_frame = frames.get_color_frame()
                
                if not color_frame:
                    print("No color frame - retrying...")
                    continue
                
                color_image = np.asanyarray(color_frame.get_data())
                
                # Handle RGB vs BGR for R200
                if color_image.shape[2] == 3 and len(color_image.shape) == 3:
                    # Check if it's RGB (R200 often outputs RGB)
                    if color_image[:,:,0].mean() < color_image[:,:,2].mean():  # Simple heuristic
                        color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)
                
                # Add status overlay
                status_text = "R200 Camera - IR Control"
                if depth_sensor and ir_options:
                    try:
                        for option, name in ir_options:
                            if option in [rs.option.emitter_enabled, rs.option.emitter_on_off]:
                                state = depth_sensor.get_option(option)
                                status_text += f" | IR: {'ON' if state else 'OFF'}"
                                break
                    except:
                        pass
                
                cv2.putText(color_image, status_text, (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                cv2.imshow('R200 Camera', color_image)
                
                key = cv2.waitKey(1) & 0xFF
                
                if key == ord('q'):
                    break
                elif key == ord('1') and depth_sensor:
                    # Turn OFF IR
                    success = False
                    for option, name in ir_options:
                        if option in [rs.option.emitter_enabled, rs.option.emitter_on_off]:
                            try:
                                depth_sensor.set_option(option, 0)
                                print(f"IR Emitter turned OFF via {name}")
                                success = True
                                break
                            except Exception as e:
                                print(f"Failed to turn off IR via {name}: {e}")
                    if not success:
                        print("No IR control method available")
                
                elif key == ord('2') and depth_sensor:
                    # Turn ON IR
                    success = False
                    for option, name in ir_options:
                        if option in [rs.option.emitter_enabled, rs.option.emitter_on_off]:
                            try:
                                depth_sensor.set_option(option, 1)
                                print(f"IR Emitter turned ON via {name}")
                                success = True
                                break
                            except Exception as e:
                                print(f"Failed to turn on IR via {name}: {e}")
                    if not success:
                        print("No IR control method available")
                
                elif key == ord('3') and depth_sensor:
                    # Decrease laser power
                    for option, name in ir_options:
                        if option == rs.option.laser_power:
                            try:
                                current = depth_sensor.get_option(option)
                                new_power = max(current - 20, 0)
                                depth_sensor.set_option(option, new_power)
                                print(f"Laser power: {new_power}")
                                break
                            except Exception as e:
                                print(f"Could not adjust laser power: {e}")
                
                elif key == ord('4') and depth_sensor:
                    # Increase laser power
                    for option, name in ir_options:
                        if option == rs.option.laser_power:
                            try:
                                current = depth_sensor.get_option(option)
                                range_info = depth_sensor.get_option_range(option)
                                new_power = min(current + 20, range_info.max)
                                depth_sensor.set_option(option, new_power)
                                print(f"Laser power: {new_power}")
                                break
                            except Exception as e:
                                print(f"Could not adjust laser power: {e}")
                
                elif key == ord('5') and color_sensor:
                    # Reduce exposure
                    if color_sensor.supports(rs.option.exposure):
                        try:
                            current_exp = color_sensor.get_option(rs.option.exposure)
                            new_exp = max(current_exp * 0.7, 1)
                            color_sensor.set_option(rs.option.exposure, new_exp)
                            print(f"Exposure reduced to {new_exp}")
                        except Exception as e:
                            print(f"Could not adjust exposure: {e}")
                
                elif key == ord('6') and color_sensor:
                    # Reset exposure
                    if color_sensor.supports(rs.option.enable_auto_exposure):
                        try:
                            color_sensor.set_option(rs.option.enable_auto_exposure, 1)
                            print("Auto exposure enabled")
                        except Exception as e:
                            print(f"Could not reset exposure: {e}")
                
                elif key == ord('s'):
                    # Save frame
                    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                    if not os.path.exists('r200_captures'):
                        os.makedirs('r200_captures')
                    
                    filename = f"r200_captures/r200_color_{timestamp}.jpg"
                    cv2.imwrite(filename, color_image)
                    print(f"Frame saved: {filename}")
                
            except Exception as e:
                print(f"Frame processing error: {e}")
                # Don't break, just continue trying
                
    except KeyboardInterrupt:
        print("\nStopping...")
    
    finally:
        try:
            pipeline.stop()
        except:
            pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    print("=== REALSENSE R200 DEDICATED CONTROL ===")
    print("Specialized script for R200 camera detection and IR control")
    print()
    
    # Check if pyrealsense2 is available
    try:
        print(f"pyrealsense2 version: {rs.__version__}")
    except:
        print("pyrealsense2 version: Unknown")
    
    input("Press Enter to start R200 detection and control...")
    r200_ir_control()
