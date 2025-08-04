# takes RGBD
# only works with D455, not R200


import pyrealsense2 as rs
import numpy as np
import cv2
import os
import json
from datetime import datetime

def save_camera_intrinsics(pipeline, timestamp):
    """Save camera intrinsics to JSON file for calibration"""
    profile = pipeline.get_active_profile()
    
    # Get color stream intrinsics
    color_stream = profile.get_stream(rs.stream.color)
    color_intrinsics = color_stream.as_video_stream_profile().get_intrinsics()
    
    # Get depth stream intrinsics
    depth_stream = profile.get_stream(rs.stream.depth)
    depth_intrinsics = depth_stream.as_video_stream_profile().get_intrinsics()
    
    # Get extrinsics (transformation between color and depth)
    extrinsics = depth_stream.get_extrinsics_to(color_stream)
    
    intrinsics_data = {
        "timestamp": timestamp,
        "color_intrinsics": {
            "width": color_intrinsics.width,
            "height": color_intrinsics.height,
            "fx": color_intrinsics.fx,
            "fy": color_intrinsics.fy,
            "ppx": color_intrinsics.ppx,
            "ppy": color_intrinsics.ppy,
            "model": str(color_intrinsics.model),
            "coeffs": color_intrinsics.coeffs
        },
        "depth_intrinsics": {
            "width": depth_intrinsics.width,
            "height": depth_intrinsics.height,
            "fx": depth_intrinsics.fx,
            "fy": depth_intrinsics.fy,
            "ppx": depth_intrinsics.ppx,
            "ppy": depth_intrinsics.ppy,
            "model": str(depth_intrinsics.model),
            "coeffs": depth_intrinsics.coeffs
        },
        "extrinsics": {
            "rotation": extrinsics.rotation,
            "translation": extrinsics.translation
        }
    }
    
    # Create directory if it doesn't exist
    if not os.path.exists('camera_calibration'):
        os.makedirs('camera_calibration')
    
    filename = f"camera_calibration/intrinsics_{timestamp}.json"
    with open(filename, 'w') as f:
        json.dump(intrinsics_data, f, indent=2, default=str)
    
    print(f"Camera intrinsics saved: {filename}")
    return filename

def save_rgbd_data(color_image, depth_image, timestamp):
    """Save synchronized RGB-D data"""
    # Create directories if they don't exist
    if not os.path.exists('saved_images'):
        os.makedirs('saved_images')
    if not os.path.exists('saved_depth'):
        os.makedirs('saved_depth')
    if not os.path.exists('saved_rgbd_pairs'):
        os.makedirs('saved_rgbd_pairs')
    
    # Save color image
    color_filename = f"saved_images/color_{timestamp}.jpg"
    cv2.imwrite(color_filename, color_image)
    
    # Save depth image as 16-bit PNG (preserves depth precision)
    depth_filename = f"saved_depth/depth_{timestamp}.png"
    cv2.imwrite(depth_filename, depth_image)
    
    # Save metadata for the RGB-D pair
    rgbd_metadata = {
        "timestamp": timestamp,
        "color_image": color_filename,
        "depth_image": depth_filename,
        "depth_scale": "Use pipeline depth scale",
        "notes": "Synchronized RGB-D capture"
    }
    
    metadata_filename = f"saved_rgbd_pairs/rgbd_{timestamp}.json"
    with open(metadata_filename, 'w') as f:
        json.dump(rgbd_metadata, f, indent=2)
    
    print(f"RGB-D pair saved: {color_filename}, {depth_filename}")
    return color_filename, depth_filename

def save_point_cloud(points, timestamp):
    """Save point cloud data as PLY file"""
    if not os.path.exists('saved_pointclouds'):
        os.makedirs('saved_pointclouds')
    
    filename = f"saved_pointclouds/pointcloud_{timestamp}.ply"
    
    # Convert to Open3D format and save
    # Note: You'll need to install open3d: pip install open3d
    try:
        import open3d as o3d
        pcd = o3d.geometry.PointCloud()
        pcd.points = o3d.utility.Vector3dVector(points)
        o3d.io.write_point_cloud(filename, pcd)
        print(f"Point cloud saved: {filename}")
    except ImportError:
        # Fallback: save as numpy array
        np.save(filename.replace('.ply', '.npy'), points)
        print(f"Point cloud saved as numpy array: {filename.replace('.ply', '.npy')}")

def main():
    # Configure depth and color streams
    pipeline = rs.pipeline()
    config = rs.config()
    
    # Enable color stream
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
    
    # Enable depth stream
    config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
    
    # Start streaming
    try:
        pipeline.start(config)
        print("RealSense camera started successfully!")
        print("Controls:")
        print("  's' - Save RGB-D pair")
        print("  'i' - Save camera intrinsics")
        print("  'p' - Save point cloud")
        print("  'c' - Save colorized depth image")
        print("  'q' - Quit")
        
        # Create a colorizer for depth visualization
        colorizer = rs.colorizer()
        
        # Create point cloud object
        pc = rs.pointcloud()
        
        # Get depth scale for point cloud generation
        depth_sensor = pipeline.get_active_profile().get_device().first_depth_sensor()
        depth_scale = depth_sensor.get_depth_scale()
        print(f"Depth scale: {depth_scale}")
        
        while True:
            # Wait for a coherent pair of frames: depth and color
            frames = pipeline.wait_for_frames()
            depth_frame = frames.get_depth_frame()
            color_frame = frames.get_color_frame()
            
            if not depth_frame or not color_frame:
                continue
            
            # Convert images to numpy arrays
            depth_image = np.asanyarray(depth_frame.get_data())
            color_image = np.asanyarray(color_frame.get_data())
            
            # Create colorized depth image for visualization
            colorized_depth = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            
            # Display images
            cv2.imshow('Color Image', color_image)
            cv2.imshow('Depth Image (Colorized)', colorized_depth)
            
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('s'):
                # Save RGB-D pair
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # Include milliseconds
                save_rgbd_data(color_image, depth_image, timestamp)
                
            elif key == ord('i'):
                # Save camera intrinsics
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                save_camera_intrinsics(pipeline, timestamp)
                
            elif key == ord('p'):
                # Save point cloud
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
                
                # Generate point cloud
                pc.map_to(color_frame)
                points = pc.calculate(depth_frame)
                vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
                
                # Filter out invalid points (0,0,0)
                valid_points = vertices[~np.all(vertices == 0, axis=1)]
                
                if len(valid_points) > 0:
                    save_point_cloud(valid_points, timestamp)
                else:
                    print("No valid points in point cloud")
                    
            elif key == ord('c'):
                # Save colorized depth image
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                if not os.path.exists('saved_depth_colorized'):
                    os.makedirs('saved_depth_colorized')
                filename = f"saved_depth_colorized/depth_colorized_{timestamp}.jpg"
                cv2.imwrite(filename, colorized_depth)
                print(f"Colorized depth image saved: {filename}")
    
    except Exception as e:
        print(f"Error: {e}")
        print("Make sure RealSense camera is connected and pyrealsense2 is installed")
        print("Install with: pip install pyrealsense2")
        
        # Check for available devices
        ctx = rs.context()
        devices = ctx.query_devices()
        if len(devices) == 0:
            print("No RealSense devices detected!")
            print("Please:")
            print("1. Connect your RealSense camera")
            print("2. Make sure USB cable is working")
            print("3. Try running: lsusb | grep Intel")
        else:
            print(f"Found {len(devices)} RealSense device(s):")
            for i, dev in enumerate(devices):
                print(f"  Device {i}: {dev.get_info(rs.camera_info.name)}")
    
    finally:
        # Stop streaming only if it was started
        try:
            pipeline.stop()
        except:
            pass
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
