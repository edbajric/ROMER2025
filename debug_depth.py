import cv2
import numpy as np
from use_calibrated_camera import CalibratedCamera

def debug_single_depth_image():
    """Debug a single depth image to understand what's going wrong"""
    
    print("=== DEBUGGING SINGLE DEPTH IMAGE ===")
    
    # Pick one depth image to analyze
    depth_file = "saved_depth/depth_20250804_153820_589.png"
    color_file = "saved_images/color_20250804_153820_589.jpg"
    
    print(f"📁 Analyzing: {depth_file}")
    
    # Load calibrated camera
    cal_cam = CalibratedCamera()
    if not cal_cam.loaded:
        print("❌ No calibration data!")
        return
    
    print(f"✅ Camera params: fx={cal_cam.fx:.1f}, fy={cal_cam.fy:.1f}")
    print(f"                  cx={cal_cam.cx:.1f}, cy={cal_cam.cy:.1f}")
    
    # Load depth image
    depth = cv2.imread(depth_file, cv2.IMREAD_UNCHANGED)
    if depth is None:
        print(f"❌ Could not load {depth_file}")
        return
    
    print(f"✅ Depth loaded: {depth.shape}, dtype: {depth.dtype}")
    print(f"📊 Depth stats: min={depth.min()}, max={depth.max()}, mean={depth[depth>0].mean():.1f}")
    
    # Test different depth scales
    scales_to_test = [0.001, 0.0001, 0.00001, 0.01]
    
    for scale in scales_to_test:
        print(f"\n🔍 Testing depth_scale = {scale}")
        
        # Sample a few pixels
        test_pixels = [
            (320, 240),  # Center
            (200, 200),  # Upper left
            (400, 300),  # Lower right
        ]
        
        for u, v in test_pixels:
            if v < depth.shape[0] and u < depth.shape[1]:
                depth_value = depth[v, u]
                if depth_value > 0:
                    depth_meters = depth_value * scale
                    X, Y, Z = cal_cam.pixel_to_camera_coordinates(u, v, depth_meters)
                    
                    print(f"  Pixel ({u},{v}): depth={depth_value} → Z={Z:.3f}m, X={X:.3f}m, Y={Y:.3f}m")
                    
                    # Check for reasonable values
                    if abs(X) > 10 or abs(Y) > 10 or Z > 10:
                        print(f"    ⚠️  Values too large! Points will be scattered")
                    elif Z < 0.1:
                        print(f"    ⚠️  Z too small! Points too close")
                    elif 0.3 < Z < 3.0 and abs(X) < 2 and abs(Y) < 2:
                        print(f"    ✅ Values look reasonable!")
    
    # Find the best scale based on reasonable Z values
    print(f"\n🎯 RECOMMENDATIONS:")
    valid_pixels = depth[depth > 0]
    if len(valid_pixels) > 0:
        avg_depth_value = valid_pixels.mean()
        
        # For typical room/object scenes, Z should be 0.5-3 meters
        target_z = 1.0  # 1 meter
        recommended_scale = target_z / avg_depth_value
        
        print(f"📐 Average depth value: {avg_depth_value:.1f}")
        print(f"💡 For ~1m distance, try: depth_scale = {recommended_scale:.6f}")
        
        # Test this scale
        print(f"\n🧪 Testing recommended scale:")
        u, v = 320, 240  # Center pixel
        depth_value = depth[v, u]
        if depth_value > 0:
            depth_meters = depth_value * recommended_scale
            X, Y, Z = cal_cam.pixel_to_camera_coordinates(u, v, depth_meters)
            print(f"   Center pixel → Z={Z:.3f}m, X={X:.3f}m, Y={Y:.3f}m")

def create_test_pointcloud():
    """Create a small test point cloud with correct scale"""
    
    print(f"\n=== CREATING TEST POINT CLOUD ===")
    
    depth_file = "saved_depth/depth_20250804_153820_589.png"
    color_file = "saved_images/color_20250804_153820_589.jpg"
    
    cal_cam = CalibratedCamera()
    depth = cv2.imread(depth_file, cv2.IMREAD_UNCHANGED)
    color = cv2.imread(color_file)
    
    if depth is None or color is None or not cal_cam.loaded:
        print("❌ Missing data")
        return
    
    # Use a reasonable scale based on analysis
    avg_depth = depth[depth > 0].mean()
    depth_scale = 1.0 / avg_depth  # This should give ~1m for average depth
    
    print(f"📐 Using depth_scale = {depth_scale:.6f}")
    
    points = []
    colors = []
    
    height, width = depth.shape
    
    # Sample every 10th pixel for a smaller test cloud
    for v in range(0, height, 10):
        for u in range(0, width, 10):
            depth_value = depth[v, u]
            
            if depth_value > 0:
                depth_meters = depth_value * depth_scale
                X, Y, Z = cal_cam.pixel_to_camera_coordinates(u, v, depth_meters)
                
                # Filter out crazy values
                if abs(X) < 5 and abs(Y) < 5 and 0.1 < Z < 5:
                    points.append([X, Y, Z])
                    
                    b, g, r = color[v, u]
                    colors.append([r, g, b])
    
    print(f"✅ Generated {len(points)} test points")
    
    if len(points) > 0:
        # Save test point cloud
        filename = "test_pointcloud.ply"
        with open(filename, 'w') as f:
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
            
            for i in range(len(points)):
                x, y, z = points[i]
                r, g, b = colors[i]
                f.write(f"{x:.6f} {y:.6f} {z:.6f} {int(r)} {int(g)} {int(b)}\n")
        
        print(f"📁 Test point cloud saved: {filename}")
        print(f"🔧 Try: meshlab {filename}")
        
        # Print some stats
        points = np.array(points)
        print(f"📊 Point cloud stats:")
        print(f"   X range: {points[:,0].min():.3f} to {points[:,0].max():.3f}")
        print(f"   Y range: {points[:,1].min():.3f} to {points[:,1].max():.3f}")
        print(f"   Z range: {points[:,2].min():.3f} to {points[:,2].max():.3f}")

if __name__ == "__main__":
    debug_single_depth_image()
    create_test_pointcloud()
