import cv2
import numpy as np
import os
from use_calibrated_camera import CalibratedCamera

def depth_image_to_pointcloud(depth_image_path, color_image_path=None, output_path=None):
    """Convert depth image to point cloud using camera calibration"""
    
    # Load calibrated camera
    cal_cam = CalibratedCamera()
    if not cal_cam.loaded:
        print("❌ No calibration data found! Please calibrate camera first.")
        return None
    
    # Load depth image
    depth_image = cv2.imread(depth_image_path, cv2.IMREAD_UNCHANGED)
    if depth_image is None:
        print(f"❌ Could not load depth image: {depth_image_path}")
        return None
    
    # Handle different depth image formats
    if len(depth_image.shape) == 3:
        # RGB depth image (R200 format) - convert to grayscale depth
        if depth_image.shape[2] == 3:
            # Convert RGB to single channel depth (use intensity)
            depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
            print(f"✓ Converted RGB depth to grayscale")
    
    # Load color image if provided
    color_image = None
    if color_image_path and os.path.exists(color_image_path):
        color_image = cv2.imread(color_image_path)
        print(f"✓ Color image loaded: {color_image_path}")
    
    print(f"✓ Depth image loaded: {depth_image.shape}")
    print(f"✓ Depth range: {depth_image.min()} - {depth_image.max()}")
    
    # Convert depth to real-world coordinates
    points = []
    colors = []
    
    height, width = depth_image.shape
    # Use data-driven scale calculation instead of fixed scale
    valid_depths = depth_image[depth_image > 0]
    if len(valid_depths) > 0:
        avg_depth_value = valid_depths.mean()
        depth_scale = 1.0 / avg_depth_value  # This gives ~1m for average depth
        print(f"📐 Calculated depth_scale = {depth_scale:.6f} (avg depth value: {avg_depth_value:.1f})")
    else:
        depth_scale = 0.001  # Fallback
        print("⚠️  No valid depth pixels, using fallback scale")
    
    print("Converting depth to 3D points...")
    
    for v in range(0, height, 1):  # Use every pixel for better quality
        for u in range(0, width, 1):
            depth_value = depth_image[v, u]
            
            if depth_value > 0:  # Valid depth
                # Convert to meters
                depth_meters = depth_value * depth_scale
                
                # Convert pixel to 3D coordinates using calibration
                X, Y, Z = cal_cam.pixel_to_camera_coordinates(u, v, depth_meters)
                
                points.append([X, Y, Z])
                
                # Add color if available
                if color_image is not None:
                    b, g, r = color_image[v, u]
                    colors.append([r, g, b])  # RGB format for PLY
                else:
                    colors.append([128, 128, 128])  # Gray
    
    points = np.array(points)
    colors = np.array(colors)
    
    print(f"✓ Generated {len(points)} 3D points")
    
    # Save as PLY file for MeshLab
    if output_path is None:
        base_name = os.path.splitext(os.path.basename(depth_image_path))[0]
        output_path = f"pointclouds/{base_name}_pointcloud.ply"
    
    os.makedirs(os.path.dirname(output_path), exist_ok=True)
    
    save_pointcloud_ply(points, colors, output_path)
    
    return points, colors

def save_pointcloud_ply(points, colors, filename):
    """Save point cloud as PLY file for MeshLab"""
    
    with open(filename, 'w') as f:
        # PLY header
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")
        
        # Point data
        for i in range(len(points)):
            x, y, z = points[i]
            r, g, b = colors[i]
            f.write(f"{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n")
    
    print(f"✓ Point cloud saved: {filename}")
    print(f"📁 Open this file in MeshLab!")

def convert_saved_depth_images():
    """Convert all saved depth images to point clouds"""
    
    depth_dirs = [
        'saved_depth',           # Your RGB-D data is here!
        'r200_rgbd',             # R200 RGB-D pairs
        'realsense_captures', 
        'output',
        'r200_captures'
    ]
    
    converted_files = []
    
    for depth_dir in depth_dirs:
        if not os.path.exists(depth_dir):
            print(f"Directory {depth_dir}/ not found, skipping...")
            continue
            
        print(f"\n🔍 Scanning {depth_dir}/...")
        
        files_in_dir = os.listdir(depth_dir)
        depth_files = [f for f in files_in_dir if f.endswith('.png') and 'depth' in f.lower()]
        
        if not depth_files:
            print(f"  No depth PNG files found in {depth_dir}/")
            continue
            
        print(f"  Found {len(depth_files)} depth images")
        
        for filename in depth_files:
            depth_path = os.path.join(depth_dir, filename)
            
            # Look for matching color image using the timestamp
            timestamp = filename.replace('depth_', '').replace('.png', '')
            color_filename = f"color_{timestamp}.jpg"
            
            # Check in same directory first (for r200_rgbd), then saved_images
            color_path = None
            
            # Try same directory first (r200_rgbd case)
            color_path_same_dir = os.path.join(depth_dir, color_filename)
            if os.path.exists(color_path_same_dir):
                color_path = color_path_same_dir
                print(f"  📸 Found matching color image: {color_filename}")
            else:
                # Try saved_images directory (saved_depth case)
                color_path_candidate = os.path.join('saved_images', color_filename)
                if os.path.exists(color_path_candidate):
                    color_path = color_path_candidate
                    print(f"  📸 Found matching color image: {color_filename}")
                else:
                    print(f"  ⚠️  No matching color image found for {filename}")
            
            print(f"  🔄 Converting: {filename}")
            try:
                points, colors = depth_image_to_pointcloud(depth_path, color_path)
                if points is not None:
                    converted_files.append(filename)
                    print(f"  ✅ Success!")
            except Exception as e:
                print(f"  ❌ Error converting {filename}: {e}")
    
    print(f"\n🎉 Converted {len(converted_files)} depth images to point clouds!")
    if len(converted_files) > 0:
        print("📁 Check the 'pointclouds/' directory")
        print("🔧 Open .ply files in MeshLab for 3D viewing and editing")
        print(f"🎯 Ready for MeshLab: {len(converted_files)} point cloud files!")
    else:
        print("❌ No files were converted. Check calibration data and file paths.")

if __name__ == "__main__":
    print("=== DEPTH TO POINT CLOUD CONVERTER ===")
    print("Convert your depth images to 3D point clouds for MeshLab")
    print()
    
    choice = input("Convert all saved depth images? (y/n): ").lower()
    
    if choice == 'y':
        convert_saved_depth_images()
    else:
        depth_file = input("Enter depth image path: ").strip()
        color_file = input("Enter color image path (optional): ").strip()
        
        if color_file == "":
            color_file = None
            
        depth_image_to_pointcloud(depth_file, color_file)
