import cv2
import numpy as np
import os
import json
import shutil
from datetime import datetime
import pyrealsense2 as rs

class RGBDCaptureSession:
    """Capture and organize RGB-D data into session folders"""
    
    def __init__(self):
        self.session_dir = None
        self.setup_session()
        self.pipeline = None
        self.config = None
        self.device_model = None
        # added for SR300 depth alignment and metadata
        self.align = None
        self.profile = None
        self.depth_scale = None
        self.color_intrinsics = None
        # added: diagnostics
        self._warned_zero_depth = False
        # NEW: display range (meters)
        self.near_m = 0.2
        self.far_m = 1.5
        # NEW: IR stream index (D4xx has two IR sensors: 1=left, 2=right)
        self.ir_index = 1
        
    def setup_session(self):
        """Create timestamped session directory"""
        main_dir = "RGBD_photos"
        os.makedirs(main_dir, exist_ok=True)
        
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(main_dir, f"session_{timestamp}")
        os.makedirs(self.session_dir, exist_ok=True)
        
        print(f" Created session: {self.session_dir}")
        
    def detect_realsense_model(self):
        """Detect which RealSense camera model is connected"""
        try:
            ctx = rs.context()
            devices = ctx.query_devices()
            
            if len(devices) == 0:
                return None, None
            
            device = devices[0]
            name = device.get_info(rs.camera_info.name)
            
            if "R200" in name:
                return device, "R200"
            elif "D435" in name or "D455" in name or "D415" in name:
                return device, "D4xx"
            elif "SR300" in name:
                return device, "SR300"
            else:
                return device, "Unknown"
        except:
            return None, None
    
    def initialize_realsense(self):
        """Initialize RealSense camera with model-specific settings"""
        device, model = self.detect_realsense_model()
        
        if not device:
            print(" No RealSense devices found!")
            return False
        
        self.device_model = model
        print(f"✓ Detected: {device.get_info(rs.camera_info.name)} ({model})")
        
        self.pipeline = rs.pipeline()
        self.config = rs.config()
        
        try:
            if model == "R200":
                # R200 specific configuration
                self.config.enable_stream(rs.stream.color, 640, 480, rs.format.rgb8, 30)
                self.config.enable_stream(rs.stream.depth, 628, 468, rs.format.z16, 30)
                # R200 IR via SDK is not generally available in librealsense2; prefer color view for dots
            elif model == "SR300":
                # SR300 (BlasterX Senz3D)
                self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                # Try IR if supported (some SR300 firmwares expose infrared)
                try:
                    self.ir_index = 1
                    self.config.enable_stream(rs.stream.infrared, self.ir_index, 640, 480, rs.format.y8, 30)
                except Exception:
                    pass
            else:
                # D4xx, etc. (D435/D455)
                self.config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
                self.config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
                # Enable one IR stream (Y8). Use index 1 (left). You can change to 2 if needed.
                self.ir_index = 1
                self.config.enable_stream(rs.stream.infrared, self.ir_index, 640, 480, rs.format.y8, 30)
            
            self.profile = self.pipeline.start(self.config)

            # Align depth to color for consistent pixel correspondence
            self.align = rs.align(rs.stream.color)

            # Configure depth sensor options (emitter/laser/AE)
            self._configure_depth_sensor()

            # Query depth scale (meters per unit) and intrinsics
            depth_sensor = self.profile.get_device().first_depth_sensor()
            try:
                self.depth_scale = float(depth_sensor.get_depth_scale())
            except Exception:
                self.depth_scale = None

            try:
                cprof = self.profile.get_stream(rs.stream.color).as_video_stream_profile()
                intr = cprof.get_intrinsics()
                self.color_intrinsics = {
                    "width": intr.width,
                    "height": intr.height,
                    "fx": intr.fx,
                    "fy": intr.fy,
                    "cx": intr.ppx,
                    "cy": intr.ppy,
                    "model": int(intr.model),
                    "coeffs": list(intr.coeffs),
                }
            except Exception:
                self.color_intrinsics = None

            print("✓ RealSense pipeline started")
            if self.depth_scale is not None:
                print(f"  Depth scale (m/unit): {self.depth_scale}")
            return True
            
        except Exception as e:
            print(f" Failed to start RealSense: {e}")
            return False

    # NEW: ensure projector/emitter and exposure are set for SR300
    def _configure_depth_sensor(self):
        try:
            dev = self.profile.get_device()
            depth_sensor = dev.first_depth_sensor()

            def set_opt(opt, val):
                try:
                    if depth_sensor.supports(opt):
                        depth_sensor.set_option(opt, float(val))
                        return True
                except Exception:
                    pass
                return False

            # Try enabling emitter/projector and reasonable laser power
            set_opt(rs.option.emitter_enabled, 1)  # may be unsupported on SR300; ignored if not
            if depth_sensor.supports(rs.option.laser_power):
                rng = depth_sensor.get_option_range(rs.option.laser_power)
                mid = 0.5 * (rng.min + rng.max)
                set_opt(rs.option.laser_power, mid)

            # Auto-exposure on
            set_opt(rs.option.enable_auto_exposure, 1)

            # Small warmup for AE/emitter to take effect
            for _ in range(15):
                self.pipeline.wait_for_frames()

        except Exception:
            pass
    
    def initialize_opencv_fallback(self):
        """Fallback to OpenCV for R200 if RealSense SDK fails"""
        print("Trying OpenCV fallback for R200...")
        
        self.color_cap = None
        self.depth_cap = None
        self.device_model = "R200_OpenCV"
        
        # Try to find color and depth streams
        for i in range(6):
            cap = cv2.VideoCapture(i)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret and len(frame.shape) == 3:  # Color image
                    if self.color_cap is None:
                        print(f"✓ Color stream found on /dev/video{i}")
                        self.color_cap = cap
                        continue
                    elif self.depth_cap is None:
                        print(f"✓ Depth stream found on /dev/video{i}")
                        self.depth_cap = cap
                        break
                cap.release()
        
        return self.color_cap is not None or self.depth_cap is not None
    
    def get_frames_realsense(self):
        """Get frames from RealSense camera (aligned depth to color)"""
        try:
            frames = self.pipeline.wait_for_frames()
            if self.align is not None:
                frames = self.align.process(frames)

            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            # NEW: IR frame (if configured)
            ir_frame = None
            try:
                ir_frame = frames.get_infrared_frame(self.ir_index)
            except Exception:
                # Some models/firmwares expose a single IR without index
                try:
                    ir_frame = frames.get_infrared_frame()
                except Exception:
                    ir_frame = None
            
            if not color_frame or not depth_frame:
                # IR alone isn’t useful without depth; return None to keep behavior
                return None, None, None
            
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())  # uint16 z16
            ir_image = np.asanyarray(ir_frame.get_data()) if ir_frame else None  # uint8 Y8

            # R200 color is RGB -> convert to BGR
            if self.device_model == "R200" and len(color_image.shape) == 3:
                color_image = cv2.cvtColor(color_image, cv2.COLOR_RGB2BGR)

            # Warn once if depth is mostly zeros (emitter off / out of range)
            if depth_image.dtype == np.uint16:
                zeros = np.count_nonzero(depth_image == 0)
                ratio = zeros / depth_image.size
                if ratio > 0.95 and not self._warned_zero_depth:
                    self._warned_zero_depth = True
                    print("Warning: depth is >95% zeros. Check emitter/laser power and range.")

            return color_image, depth_image, ir_image
            
        except Exception as e:
            print(f"Frame error: {e}")
            return None, None, None
    
    def get_frames_opencv(self):
        """Get frames from OpenCV (fallback). Attempts to read true depth; otherwise IR only."""
        color_frame = None
        depth_frame = None
        ir_frame = None
        
        if hasattr(self, 'color_cap') and self.color_cap:
            ret, frm = self.color_cap.read()
            if ret and frm is not None:
                # If YUYV (2 channels), convert to BGR
                if len(frm.shape) == 3 and frm.shape[2] == 2:
                    color_frame = cv2.cvtColor(frm, cv2.COLOR_YUV2BGR_YUYV)
                elif len(frm.shape) == 3 and frm.shape[2] == 3:
                    color_frame = frm
                elif len(frm.shape) == 2:
                    color_frame = cv2.cvtColor(frm, cv2.COLOR_GRAY2BGR)
        
        if hasattr(self, 'depth_cap') and self.depth_cap:
            ret, frm = self.depth_cap.read()
            if ret and frm is not None:
                # If 3-channel, treat as grayscale (IR), not true depth
                if len(frm.shape) == 3 and frm.shape[2] == 3:
                    depth_frame = cv2.cvtColor(frm, cv2.COLOR_BGR2GRAY)
                else:
                    depth_frame = frm

                # If it's not uint16, it's IR (not true depth)
                if depth_frame.dtype != np.uint16:
                    # Try to detect 16-bit packed data misreported; otherwise keep as 8-bit IR
                    if depth_frame.ndim == 2 and depth_frame.max() > 255:
                        depth_frame = depth_frame.astype(np.uint16)
        
        return color_frame, depth_frame, ir_frame
    
    def save_rgbd_triplet(self, color_image, depth_image):
        """Save RGB-D triplet (color, depth, metadata) in session folder"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]  # Include milliseconds
        
        # Save color image
        color_filename = f"color_{timestamp}.jpg"
        color_path = os.path.join(self.session_dir, color_filename)
        cv2.imwrite(color_path, color_image)
        
        # Save depth image
        depth_filename = f"depth_{timestamp}.png"
        depth_path = os.path.join(self.session_dir, depth_filename)
        # Preserve 16-bit if available; otherwise this is likely IR and not usable for point clouds
        if depth_image.dtype == np.uint16:
            cv2.imwrite(depth_path, depth_image)
        else:
            # Ensure single-channel 8-bit save (IR)
            if depth_image.ndim == 3 and depth_image.shape[2] == 3:
                depth_image = cv2.cvtColor(depth_image, cv2.COLOR_BGR2GRAY)
            cv2.imwrite(depth_path, depth_image)
        
        # Create metadata JSON
        metadata = {
            "timestamp": timestamp,
            "session_dir": self.session_dir,
            "color_image": color_filename,
            "depth_image": depth_filename,
            "camera_model": self.device_model,
            "color_format": "BGR" if self.device_model != "R200" else "RGB",
            "depth_format": "16-bit PNG" if depth_image.dtype == np.uint16 else "8-bit PNG",
            "depth_units_meters": self.depth_scale if self.depth_scale is not None else None,
            "color_intrinsics": self.color_intrinsics,
            "notes": f"RGB-D triplet captured with {self.device_model}",
            "depth_is_true_z16": bool(depth_image.dtype == np.uint16),
        }
        
        json_filename = f"rgbd_{timestamp}.json"
        json_path = os.path.join(self.session_dir, json_filename)
        
        with open(json_path, 'w') as f:
            json.dump(metadata, f, indent=2)
        
        print(f"✓ RGB-D triplet saved: {timestamp}")
        return {
            'color': color_path,
            'depth': depth_path,
            'metadata': json_path
        }
    
    def create_session_summary(self, captured_count):
        """Create session summary"""
        summary = {
            "session_timestamp": datetime.now().strftime("%Y%m%d_%H%M%S"),
            "session_directory": self.session_dir,
            "camera_model": self.device_model,
            "total_captures": captured_count,
            "capture_format": "RGB-D triplets (color.jpg + depth.png + metadata.json)",
            "ready_for_conversion": True,
            "next_steps": [
                f"Run: python convert_session_to_pointclouds.py {self.session_dir}",
                f"Or: meshlab {self.session_dir}/pointclouds/*.ply"
            ]
        }
        
        summary_path = os.path.join(self.session_dir, "session_summary.json")
        with open(summary_path, 'w') as f:
            json.dump(summary, f, indent=2)
        
        print(f" Session summary saved: {summary_path}")
    
    # NEW: helpers to access depth sensor and adjust emitter/laser
    def _get_depth_sensor(self):
        try:
            return self.profile.get_device().first_depth_sensor()
        except Exception:
            return None

    def _toggle_emitter(self):
        s = self._get_depth_sensor()
        if not s:
            print("No depth sensor.")
            return
        try:
            if s.supports(rs.option.emitter_enabled):
                v = s.get_option(rs.option.emitter_enabled)
                s.set_option(rs.option.emitter_enabled, 0.0 if v > 0.5 else 1.0)
                print(f"Emitter {'OFF' if v > 0.5 else 'ON'}")
            else:
                print("Emitter toggle not supported on this device.")
        except Exception as e:
            print(f"Emitter toggle failed: {e}")

    def _adjust_laser_power(self, delta_frac=0.1):
        s = self._get_depth_sensor()
        if not s:
            print("No depth sensor.")
            return
        try:
            if s.supports(rs.option.laser_power):
                rng = s.get_option_range(rs.option.laser_power)
                cur = s.get_option(rs.option.laser_power)
                step = max((rng.max - rng.min) * abs(delta_frac), rng.step if rng.step > 0 else 1.0)
                newv = np.clip(cur + (step if delta_frac > 0 else -step), rng.min, rng.max)
                s.set_option(rs.option.laser_power, float(newv))
                print(f"Laser power: {newv:.1f} (range {rng.min:.1f}-{rng.max:.1f})")
            else:
                print("Laser power not supported on this device.")
        except Exception as e:
            print(f"Laser power adjust failed: {e}")
    
    def run_capture_session(self):
        """Main capture loop"""
        # Try RealSense SDK first
        if not self.initialize_realsense():
            # Fallback to OpenCV
            if not self.initialize_opencv_fallback():
                print(" No camera initialization method worked!")
                return
        
        print(f"\n RGB-D Capture Session Started")
        print(f"Session folder: {self.session_dir}")
        print(f" Camera: {self.device_model}")
        print("\nControls:")
        print("  's' - Save RGB-D triplet")
        print("  'i' - Save IR frame (if available)")
        print("  'e' - toggle emitter (if supported)")
        print("  '-'/'=' - laser power down/up")
        print("  ','/'.' - near range -/+ (m)")
        print("  '['/']' - far range -/+ (m)")
        print("  'q' - Quit and finalize session")
        
        capture_count = 0
        
        while True:
            # Get frames based on method
            if self.pipeline:
                color_image, depth_image, ir_image = self.get_frames_realsense()
            else:
                color_image, depth_image, ir_image = self.get_frames_opencv()
            
            # Display color
            if color_image is not None:
                cv2.imshow('Color', color_image)

            # Display depth (colorized for viewing; raw z16 is saved)
            if depth_image is not None:
                dvis = self._depth_for_display(depth_image)
                if dvis is not None:
                    if depth_image.dtype == np.uint16:
                        valid_ratio = 1.0 - (np.count_nonzero(depth_image == 0) / depth_image.size)
                        dmin = int(depth_image[depth_image > 0].min()) if np.any(depth_image > 0) else 0
                        dmax = int(depth_image.max())
                        cv2.putText(dvis, f"valid: {valid_ratio*100:.1f}%  z16 min/max: {dmin}/{dmax}",
                                    (10, 24), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255,255,255), 2)
                    cv2.imshow('Depth', dvis)

            # NEW: Display IR (grayscale)
            if ir_image is not None:
                cv2.imshow(f'IR (Y8) idx {self.ir_index}', ir_image)

            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('s'):
                if color_image is not None and depth_image is not None:
                    self.save_rgbd_triplet(color_image, depth_image)
                    capture_count += 1
                else:
                    print(" Cannot save - missing color or depth frame")
            elif key == ord('i'):
                self.save_ir_frame(ir_image)
            elif key == ord('e'):
                self._toggle_emitter()
            elif key == ord('=') or key == ord('+'):
                self._adjust_laser_power(+0.1)
            elif key == ord('-') or key == ord('_'):
                self._adjust_laser_power(-0.1)
            elif key == ord('['):
                self.far_m = max(self.near_m + 0.05, self.far_m - 0.1)
                print(f"far: {self.far_m:.2f} m")
            elif key == ord(']'):
                self.far_m = min(5.0, self.far_m + 0.1)
                print(f"far: {self.far_m:.2f} m")
            elif key == ord(','):
                self.near_m = max(0.05, self.near_m - 0.05)
                print(f"near: {self.near_m:.2f} m")
            elif key == ord('.'):
                self.near_m = min(self.far_m - 0.05, self.near_m + 0.05)
                print(f"near: {self.near_m:.2f} m")
        
        # Finalize session
        self.create_session_summary(capture_count)
        print(f"\n Session completed!")
        print(f"Captured {capture_count} RGB-D triplets")
        print(f" Saved to: {self.session_dir}")
        
        if capture_count > 0:
            print(f"\n Next steps:")
            print(f"1. Convert to point clouds: python convert_session_to_pointclouds.py {self.session_dir}")
            print(f"2. View in MeshLab: meshlab {self.session_dir}/pointclouds/*.ply")
        
        # Cleanup
        self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        if self.pipeline:
            try:
                self.pipeline.stop()
            except:
                pass
        
        if hasattr(self, 'color_cap') and self.color_cap:
            self.color_cap.release()
        if hasattr(self, 'depth_cap') and self.depth_cap:
            self.depth_cap.release()
        
        cv2.destroyAllWindows()

    # Replace the display helper with range-aware visualization
    def _depth_for_display(self, depth_img):
        if depth_img is None:
            return None

        if depth_img.dtype == np.uint16:
            d = depth_img.astype(np.float32)
            mask = d > 0

            # Use adjustable near/far if scale known; otherwise percentile
            if self.depth_scale:
                d_m = d * self.depth_scale
                near, far = max(0.0, self.near_m), max(self.near_m + 0.05, self.far_m)
                d_m = np.clip(d_m, near, far)
                if mask.any():
                    d_norm = np.zeros_like(d_m, dtype=np.float32)
                    d_norm[mask] = (d_m[mask] - near) / max(1e-6, (far - near))
                    d8 = (d_norm * 255).astype(np.uint8)
                else:
                    return None
            else:
                if not mask.any():
                    return None
                vals = d[mask]
                lo = np.percentile(vals, 5)
                hi = np.percentile(vals, 95)
                if hi <= lo:
                    return None
                d_clip = np.clip(d, lo, hi)
                d_norm = np.zeros_like(d_clip, dtype=np.float32)
                d_norm[mask] = (d_clip[mask] - lo) / (hi - lo)
                d8 = (d_norm * 255).astype(np.uint8)

            return cv2.applyColorMap(d8, cv2.COLORMAP_JET)

        # IR or 8-bit stream
        if depth_img.ndim == 3 and depth_img.shape[2] == 3:
            depth_img = cv2.cvtColor(depth_img, cv2.COLOR_BGR2GRAY)
        d8 = depth_img if depth_img.dtype == np.uint8 else cv2.convertScaleAbs(depth_img)
        return cv2.applyColorMap(d8, cv2.COLORMAP_JET)

def main():
    print("=== RGB-D CAPTURE & ORGANIZE SESSION ===")
    print("Captures RGB-D data and organizes into timestamped session folders")
    print("Supports RealSense D455, R200, and OpenCV fallback")
    print()
    
    session = RGBDCaptureSession()
    session.run_capture_session()

if __name__ == "__main__":
    main()