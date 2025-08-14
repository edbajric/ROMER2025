import cv2
import numpy as np
import matplotlib.pyplot as plt
import os

def analyze_depth_quality():
    """Analyze your depth images to find quality issues"""
    
    print("=== DEPTH DATA QUALITY ANALYSIS ===")
    
    # Check a few depth images
    depth_files = []
    if os.path.exists('saved_depth'):
        depth_files = [f for f in os.listdir('saved_depth') if f.endswith('.png')][:3]
    
    if not depth_files:
        print("❌ No depth files found!")
        return
    
    for i, filename in enumerate(depth_files):
        depth_path = os.path.join('saved_depth', filename)
        print(f"\n📊 Analyzing: {filename}")
        
        # Load depth image
        depth = cv2.imread(depth_path, cv2.IMREAD_UNCHANGED)
        if depth is None:
            print(f"❌ Could not load {filename}")
            continue
            
        # Basic statistics
        valid_pixels = depth[depth > 0]
        total_pixels = depth.shape[0] * depth.shape[1]
        valid_percentage = len(valid_pixels) / total_pixels * 100
        
        print(f"  📏 Resolution: {depth.shape}")
        print(f"  📈 Depth range: {depth.min()} - {depth.max()}")
        print(f"  ✅ Valid pixels: {valid_percentage:.1f}%")
        print(f"  📊 Valid depth range: {valid_pixels.min()} - {valid_pixels.max()}")
        
        # Check for common issues
        issues = []
        
        if valid_percentage < 30:
            issues.append("⚠️  LOW COVERAGE - Most pixels have no depth data")
        
        if depth.max() < 1000:
            issues.append("⚠️  VERY CLOSE RANGE - Objects might be too close")
        
        if depth.max() > 10000:
            issues.append("⚠️  VERY FAR RANGE - Some objects might be too far")
        
        if len(np.unique(valid_pixels)) < 100:
            issues.append("⚠️  LOW PRECISION - Depth values are quantized")
        
        # Check for IR emitter issues
        zero_pixels = np.sum(depth == 0) / total_pixels * 100
        if zero_pixels > 70:
            issues.append("🔴 IR EMITTER ISSUE - Too many invalid pixels (IR might be off)")
        
        if issues:
            print("  🚨 ISSUES FOUND:")
            for issue in issues:
                print(f"    {issue}")
        else:
            print("  ✅ Depth data looks good!")
        
        # Save analysis image
        if i == 0:  # Just analyze the first one visually
            plt.figure(figsize=(15, 5))
            
            # Original depth
            plt.subplot(1, 3, 1)
            plt.imshow(depth, cmap='jet')
            plt.title('Raw Depth Image')
            plt.colorbar()
            
            # Valid pixels only
            plt.subplot(1, 3, 2)
            masked_depth = depth.copy()
            masked_depth[depth == 0] = np.nan
            plt.imshow(masked_depth, cmap='jet')
            plt.title('Valid Depth Only')
            plt.colorbar()
            
            # Histogram
            plt.subplot(1, 3, 3)
            plt.hist(valid_pixels, bins=50, alpha=0.7)
            plt.title('Depth Value Distribution')
            plt.xlabel('Depth Value')
            plt.ylabel('Frequency')
            
            plt.tight_layout()
            plt.savefig('depth_analysis.png', dpi=150)
            print(f"  📊 Analysis saved to: depth_analysis.png")
    
    print(f"\n🔧 RECOMMENDATIONS:")
    print("1. If low coverage: Turn ON IR emitter or improve lighting")
    print("2. If objects too close: Move camera back (>0.5m)")
    print("3. If objects too far: Move camera closer (<3m for R200)")
    print("4. If IR issues: Use realsense_universal_control.py to turn ON emitter")

def check_calibration_quality():
    """Check if calibration is causing the issue"""
    
    print(f"\n=== CALIBRATION QUALITY CHECK ===")
    
    try:
        from use_calibrated_camera import CalibratedCamera
        cal_cam = CalibratedCamera()
        
        if not cal_cam.loaded:
            print("❌ No calibration data loaded!")
            return
        
        print(f"✅ Calibration loaded successfully")
        print(f"📏 Focal length: fx={cal_cam.fx:.1f}, fy={cal_cam.fy:.1f}")
        print(f"📍 Principal point: cx={cal_cam.cx:.1f}, cy={cal_cam.cy:.1f}")
        
        # Check for reasonable values (640x480 camera)
        issues = []
        
        if cal_cam.fx < 300 or cal_cam.fx > 800:
            issues.append(f"⚠️  Unusual focal length fx={cal_cam.fx:.1f} (expected ~400-600)")
        
        if abs(cal_cam.cx - 320) > 100:
            issues.append(f"⚠️  Principal point cx={cal_cam.cx:.1f} far from center (expected ~320)")
        
        if abs(cal_cam.cy - 240) > 100:
            issues.append(f"⚠️  Principal point cy={cal_cam.cy:.1f} far from center (expected ~240)")
        
        if hasattr(cal_cam, 'reprojection_error'):
            error = cal_cam.reprojection_error
            if isinstance(error, (int, float)):
                if error > 1.0:
                    issues.append(f"⚠️  High reprojection error: {error:.3f} (should be < 0.5)")
                else:
                    print(f"✅ Good reprojection error: {error:.3f}")
        
        if issues:
            print("🚨 CALIBRATION ISSUES:")
            for issue in issues:
                print(f"  {issue}")
            print("💡 Consider recalibrating with more images")
        else:
            print("✅ Calibration parameters look reasonable")
            
    except Exception as e:
        print(f"❌ Error checking calibration: {e}")

def suggest_fixes():
    """Suggest specific fixes based on analysis"""
    
    print(f"\n=== QUICK FIXES TO TRY ===")
    print("1. 🔧 ADJUST DEPTH SCALE:")
    print("   Edit depth_to_pointcloud.py line 37:")
    print("   Try: depth_scale = 0.0001  # if points too far")
    print("   Try: depth_scale = 0.01    # if points too close")
    
    print(f"\n2. 🎯 IMPROVE POINT DENSITY:")
    print("   Edit depth_to_pointcloud.py line 41:")
    print("   Change: for v in range(0, height, 1):  # Use every pixel")
    print("           for u in range(0, width, 1):")
    
    print(f"\n3. 🔴 FIX IR EMITTER:")
    print("   python3 realsense_universal_control.py")
    print("   Press '2' to turn ON IR emitter")
    
    print(f"\n4. 📏 RECALIBRATE IF NEEDED:")
    print("   python3 camera_calibration.py")
    print("   Take 20+ good checkerboard images")
    
    print(f"\n5. 🎨 MESHLAB VISUALIZATION:")
    print("   In MeshLab:")
    print("   - Render → Show Points (not mesh)")
    print("   - View → Reset Trackball")
    print("   - Filters → Point Set → Point Cloud Simplification")

if __name__ == "__main__":
    try:
        import matplotlib.pyplot as plt
        has_matplotlib = True
    except ImportError:
        print("📊 Install matplotlib for visual analysis: pip install matplotlib")
        has_matplotlib = False
    
    analyze_depth_quality()
    check_calibration_quality() 
    suggest_fixes()
    
    if has_matplotlib:
        print(f"\n📈 Visual analysis saved to depth_analysis.png")
