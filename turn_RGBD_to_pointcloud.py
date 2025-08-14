import os
import sys
import json
import cv2
import numpy as np
from use_calibrated_camera import CalibratedCamera
from datetime import datetime

def convert_session_to_pointclouds(session_dir):
    """Convert an organized RGB-D session to point clouds"""
    
    # Handle both relative and absolute paths
    if not os.path.isabs(session_dir):
        session_dir = os.path.join("/home/kovan/3d-cameras", session_dir)
    
    if not os.path.exists(session_dir):
        print(f" Session directory not found: {session_dir}")
        return
    
    print(f" Converting session: {session_dir}")
    
    # Load calibration (ensure we look in the right place)
    cal_cam = CalibratedCamera()
    if not cal_cam.loaded:
        print(" No calibration data found! Please calibrate camera first.")
        print("   Run: python camera_calibration.py")
        return
    
    # Find all RGB-D triplets in session
    json_files = [f for f in os.listdir(session_dir) if f.startswith('rgbd_') and f.endswith('.json')]
    
    if not json_files:
        print(" No RGB-D JSON files found in session!")
        print(f"   Looking in: {session_dir}")
        print("   Expected files: rgbd_TIMESTAMP.json")
        return
    
    print(f"🔍 Found {len(json_files)} RGB-D triplets to convert")
    
    # Create pointclouds subdirectory
    pointclouds_dir = os.path.join(session_dir, "pointclouds")
    os.makedirs(pointclouds_dir, exist_ok=True)
    
    converted_count = 0
    total_points = 0
    
    for json_file in sorted(json_files):
        json_path = os.path.join(session_dir, json_file)
        
        try:
            # Read metadata
            with open(json_path, 'r') as f:
                rgbd_data = json.load(f)
            
            timestamp_str = rgbd_data.get("timestamp", "unknown")
            
            # Handle both old format (with paths) and new format (just filenames)
            if "color_image" in rgbd_data:
                color_file = rgbd_data["color_image"]
                depth_file = rgbd_data["depth_image"]
                
                # If they're just filenames (new format), use them directly
                if not os.path.sep in color_file:
                    color_path = os.path.join(session_dir, color_file)
                    depth_path = os.path.join(session_dir, depth_file)
                else:
                    # Old format with full paths - convert to session-relative
                    color_path = os.path.join(session_dir, os.path.basename(color_file))
                    depth_path = os.path.join(session_dir, os.path.basename(depth_file))
            else:
                # Fallback: construct filenames from timestamp
                color_file = f"color_{timestamp_str}.jpg"
                depth_file = f"depth_{timestamp_str}.png"
                color_path = os.path.join(session_dir, color_file)
                depth_path = os.path.join(session_dir, depth_file)
            
            if not os.path.exists(color_path) or not os.path.exists(depth_path):
                print(f"  Missing files for {timestamp_str}")
                print(f"    Color: {color_path} {'✓' if os.path.exists(color_path) else '✗'}")
                print(f"    Depth: {depth_path} {'✓' if os.path.exists(depth_path) else '✗'}")
                continue
            
            print(f"🔄 Converting: {timestamp_str}")
            
            # Convert to point cloud
            points, colors = rgbd_to_pointcloud(
                depth_path, color_path, cal_cam, timestamp_str
            )
            
            if points is not None and len(points) > 0:
                # Save point cloud
                ply_filename = f"pointcloud_{timestamp_str}.ply"
                ply_path = os.path.join(pointclouds_dir, ply_filename)
                
                save_pointcloud_ply(points, colors, ply_path)
                
                converted_count += 1
                total_points += len(points)
                print(f"✓ Saved: {ply_filename} ({len(points):,} points)")
            else:
                print(f" Failed to generate points for {timestamp_str}")
                
        except Exception as e:
            print(f" Error converting {json_file}: {e}")
            import traceback
            traceback.print_exc()
    
    # Create conversion summary
    conversion_summary = {
        "conversion_timestamp": datetime.now().strftime("%Y%m%d_%H%M%S"),
        "session_dir": session_dir,
        "pointclouds_dir": pointclouds_dir,
        "converted_triplets": converted_count,
        "total_point_clouds": converted_count,
        "total_points": total_points,
        "average_points_per_cloud": total_points // max(converted_count, 1),
        "calibration_used": {
            "focal_length_x": float(cal_cam.camera_matrix[0,0]),
            "focal_length_y": float(cal_cam.camera_matrix[1,1]),
            "principal_point_x": float(cal_cam.camera_matrix[0,2]),
            "principal_point_y": float(cal_cam.camera_matrix[1,2])
        }
    }
    
    summary_path = os.path.join(pointclouds_dir, "conversion_summary.json")
    with open(summary_path, 'w') as f:
        json.dump(conversion_summary, f, indent=2)
    
    print(f"\n Conversion Complete!")
    print(f" Converted: {converted_count} RGB-D triplets")
    print(f" Total points: {total_points:,}")
    print(f" Point clouds saved to: {pointclouds_dir}")
    print(f" Summary: {summary_path}")
    
    # Instructions for MeshLab
    print(f"\n Open in MeshLab:")
    print(f"meshlab {pointclouds_dir}/*.ply")
    
    return pointclouds_dir

def rgbd_to_pointcloud(depth_path, color_path, cal_cam, timestamp):
    """Convert RGB-D pair to point cloud"""
    
    # Load images
    depth_image = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
    color_image = cv2.imread(color_path)
    
    if depth_image is None:
        print(f" Could not load depth image: {depth_path}")
        return None, None
    
    if color_image is None:
        print(f" Could not load color image: {color_path}")
        return None, None
    
    print(f"   📸 Images loaded: {depth_image.shape} depth, {color_image.shape} color")
    
    # Handle depth format
    if len(depth_image.shape) == 3:
        print("    Converting color depth image to grayscale")
        depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
    
    # Calculate adaptive depth scale
    valid_depths = depth_image[depth_image > 0]
    if len(valid_depths) == 0:
        print(f"  No valid depth data for {timestamp}")
        return None, None
    
    avg_depth_value = valid_depths.mean()
    depth_scale = 1.0 / avg_depth_value  # Adaptive scaling
    
    print(f"    Depth scale: {depth_scale:.6f} (avg: {avg_depth_value:.1f})")
    
    # Convert to 3D points
    points = []
    colors = []
    
    height, width = depth_image.shape
    
    # Use every pixel for high quality (can change to range(0, height, 2) for speed)
    for v in range(0, height, 1):
        for u in range(0, width, 1):
            depth_value = depth_image[v, u]
            
            if depth_value > 0:  # Valid depth
                # Convert to meters
                depth_meters = depth_value * depth_scale
                
                # Convert to 3D coordinates
                X, Y, Z = cal_cam.pixel_to_camera_coordinates(u, v, depth_meters)
                
                points.append([X, Y, Z])
                
                # Get color
                b, g, r = color_image[v, u]
                colors.append([r, g, b])  # RGB for PLY
    
    return np.array(points), np.array(colors)

def save_pointcloud_ply(points, colors, filename):
    """Save point cloud as PLY file"""
    
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

def find_latest_session():
    """Find the most recent session directory"""
    rgbd_photos_dir = "/home/kovan/3d-cameras/RGBD_photos"
    
    if not os.path.exists(rgbd_photos_dir):
        print(f" RGBD_photos directory not found: {rgbd_photos_dir}")
        return None
    
    sessions = [d for d in os.listdir(rgbd_photos_dir) if d.startswith("session_")]
    if not sessions:
        print(f"No session folders found in {rgbd_photos_dir}")
        return None
    
    sessions.sort(reverse=True)  # Most recent first
    latest_session = os.path.join(rgbd_photos_dir, sessions[0])
    return latest_session

def list_available_sessions():
    """List all available sessions"""
    rgbd_photos_dir = "/home/kovan/3d-cameras/RGBD_photos"
    
    if not os.path.exists(rgbd_photos_dir):
        print(f"RGBD_photos directory not found: {rgbd_photos_dir}")
        return []
    
    sessions = [d for d in os.listdir(rgbd_photos_dir) if d.startswith("session_")]
    sessions.sort(reverse=True)  # Most recent first
    
    if sessions:
        print(f"\n Available sessions in {rgbd_photos_dir}:")
        for i, session in enumerate(sessions):
            session_path = os.path.join(rgbd_photos_dir, session)
            # Count RGB-D triplets
            try:
                json_files = [f for f in os.listdir(session_path) if f.startswith('rgbd_') and f.endswith('.json')]
                triplet_count = len(json_files)
                print(f"  {i+1}. {session} ({triplet_count} RGB-D triplets)")
            except:
                print(f"  {i+1}. {session} (error reading)")
    
    return [os.path.join(rgbd_photos_dir, s) for s in sessions]

if __name__ == "__main__":
    print("=== RGB-D SESSION TO POINT CLOUDS CONVERTER ===")
    print(f"Working with: /home/kovan/3d-cameras/RGBD_photos")
    
    if len(sys.argv) > 1:
        # Session directory provided as argument
        session_dir = sys.argv[1]
        
        # Handle various input formats
        if session_dir.startswith("session_"):
            # Just session name provided
            session_dir = f"/home/kovan/3d-cameras/RGBD_photos/{session_dir}"
        elif not session_dir.startswith("/"):
            # Relative path provided
            session_dir = f"/home/kovan/3d-cameras/RGBD_photos/{session_dir}"
    else:
        # Find latest session
        available_sessions = list_available_sessions()
        
        if not available_sessions:
            print(" No sessions found! Capture some RGB-D data first.")
            print("   Run: python rgbd_capture_and_organize.py")
            sys.exit(1)
        
        session_dir = available_sessions[0]  # Use latest
        print(f"🔍 Using latest session: {os.path.basename(session_dir)}")
    
    pointclouds_dir = convert_session_to_pointclouds(session_dir)
    
    if pointclouds_dir:
        print(f"\n Ready for MeshLab!")
        print(f"Command: meshlab {pointclouds_dir}/*.ply")
        print(f"Or open individual files: ls {pointclouds_dir}/*.ply")