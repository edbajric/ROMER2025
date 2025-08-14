import cv2
import numpy as np
import os
import json
from datetime import datetime

class RGBDCaptureSession:
    """ZR300 RGB-D capture using OpenCV/V4L2"""

    def __init__(self, width=640, height=480, fps=30, scan_max=16):
        self.session_dir = None
        self.setup_session()

        self.width = width
        self.height = height
        self.fps = fps
        self.scan_max = scan_max

        self.device_model = "RealSense_ZR300_OpenCV"
        self.color_cap = None
        self.depth_cap = None
        self.color_device = None
        self.depth_device = None

    def setup_session(self):
        main_dir = "RGBD_photos"
        os.makedirs(main_dir, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(main_dir, f"session_{timestamp}")
        os.makedirs(self.session_dir, exist_ok=True)
        print(f"Created session: {self.session_dir}")

    def _open_cap(self, idx, fourcc=None):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None
        if fourcc is not None:
            cap.set(cv2.CAP_PROP_FOURCC, fourcc)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        # warm up
        for _ in range(4):
            cap.read()
        return cap if cap.isOpened() else None

    def _classify_node(self, idx):
        cap = self._open_cap(idx, None)
        if cap is None:
            return "unknown"
        role = "unknown"
        for _ in range(6):
            ret, frm = cap.read()
            if not ret or frm is None:
                continue
            if len(frm.shape) == 2:
                role = "depth"  # GREY/Y16 appears as single-channel
                break
            if len(frm.shape) == 3 and frm.shape[2] == 2:
                role = "color"  # YUYV reported as 2-channel
                break
            if len(frm.shape) == 3 and frm.shape[2] == 3:
                role = "color"  # BGR/MJPG decoded by OpenCV
                break
        cap.release()
        return role

    def detect_zr300_devices(self):
        color_candidates = []
        depth_candidates = []
        print("Scanning /dev/video* for ZR300 streams...")
        for i in range(self.scan_max):
            dev = f"/dev/video{i}"
            if not os.path.exists(dev):
                continue
            role = self._classify_node(i)
            if role == "color":
                color_candidates.append(i)
                print(f"  Color candidate: {dev}")
            elif role == "depth":
                depth_candidates.append(i)
                print(f"  Depth/IR candidate: {dev}")
            else:
                print(f"  Unknown: {dev}")
        return color_candidates, depth_candidates

    def setup_zr300_streams(self):
        color_cand, depth_cand = self.detect_zr300_devices()
        if not color_cand and not depth_cand:
            print("No ZR300-like video nodes found.")
            return False

        # Open color: try MJPG then YUYV then default
        for dev in color_cand:
            for fc, tag in [
                (cv2.VideoWriter_fourcc('M','J','P','G'), "MJPG"),
                (cv2.VideoWriter_fourcc('Y','U','Y','V'), "YUYV"),
                (None, "default"),
            ]:
                cap = self._open_cap(dev, fc)
                if cap is None:
                    continue
                ret, frm = cap.read()
                if ret and frm is not None:
                    self.color_cap = cap
                    self.color_device = dev
                    print(f"Color stream: /dev/video{dev} ({tag})")
                    break
                cap.release()
            if self.color_cap:
                break

        # Open depth: try Y16 then GREY then default, not colliding with color
        for dev in depth_cand:
            if dev == self.color_device:
                continue
            for fc, tag in [
                (cv2.VideoWriter_fourcc('Y','1','6',' '), "Y16"),
                (cv2.VideoWriter_fourcc('G','R','E','Y'), "GREY"),
                (None, "default"),
            ]:
                cap = self._open_cap(dev, fc)
                if cap is None:
                    continue
                ret, frm = cap.read()
                if ret and frm is not None:
                    self.depth_cap = cap
                    self.depth_device = dev
                    print(f"Depth stream: /dev/video{dev} ({tag})")
                    break
                cap.release()
            if self.depth_cap:
                break

        ok = (self.color_cap is not None) or (self.depth_cap is not None)
        if ok:
            print("ZR300 streams initialized.")
            print(f"  Color: /dev/video{self.color_device if self.color_device is not None else 'N/A'}")
            print(f"  Depth: /dev/video{self.depth_device if self.depth_device is not None else 'N/A'}")
        else:
            print("Failed to initialize ZR300 streams.")
        return ok

    def get_zr300_frames(self):
        color_image = None
        depth_image = None

        if self.color_cap:
            ret, frm = self.color_cap.read()
            if ret and frm is not None:
                # YUYV -> BGR only if 2-channel
                if len(frm.shape) == 3 and frm.shape[2] == 2:
                    color_image = cv2.cvtColor(frm, cv2.COLOR_YUV2BGR_YUYV)
                elif len(frm.shape) == 3 and frm.shape[2] == 3:
                    color_image = frm
                elif len(frm.shape) == 2:
                    color_image = cv2.cvtColor(frm, cv2.COLOR_GRAY2BGR)

        if self.depth_cap:
            ret, frm = self.depth_cap.read()
            if ret and frm is not None:
                # If 3-channel, convert to grayscale
                if len(frm.shape) == 3 and frm.shape[2] == 3:
                    depth_image = cv2.cvtColor(frm, cv2.COLOR_BGR2GRAY)
                else:
                    depth_image = frm
                # Keep uint16 if it looks like 16-bit
                if depth_image.dtype != np.uint16 and depth_image.ndim == 2 and depth_image.max() > 255:
                    depth_image = depth_image.astype(np.uint16)

        return color_image, depth_image

    def _depth_for_display(self, depth_image):
        if depth_image is None:
            return None
        if depth_image.dtype == np.uint16:
            scale = 255.0 / max(1, int(depth_image.max()))
            d8 = cv2.convertScaleAbs(depth_image, alpha=scale)
        else:
            d8 = depth_image if depth_image.dtype == np.uint8 else cv2.convertScaleAbs(depth_image)
        return cv2.applyColorMap(d8, cv2.COLORMAP_JET)

    def save_rgbd_triplet(self, color_image, depth_image):
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        color_filename = f"color_{timestamp}.jpg"
        color_path = os.path.join(self.session_dir, color_filename)
        cv2.imwrite(color_path, color_image)

        depth_filename = f"depth_{timestamp}.png"
        depth_path = os.path.join(self.session_dir, depth_filename)
        cv2.imwrite(depth_path, depth_image)

        metadata = {
            "timestamp": timestamp,
            "session_dir": self.session_dir,
            "color_image": color_filename,
            "depth_image": depth_filename,
            "camera_model": self.device_model,
            "color_format": "BGR",
            "depth_format": "16-bit PNG" if depth_image.dtype == np.uint16 else "8-bit PNG",
            "color_device": self.color_device,
            "depth_device": self.depth_device,
        }

        json_filename = f"rgbd_{timestamp}.json"
        json_path = os.path.join(self.session_dir, json_filename)
        with open(json_path, 'w') as f:
            json.dump(metadata, f, indent=2)

        print(f"Saved RGB-D triplet: {timestamp}")
        return {"color": color_path, "depth": depth_path, "metadata": json_path}

    def create_session_summary(self, captured_count):
        summary = {
            "session_timestamp": datetime.now().strftime("%Y%m%d_%H%M%S"),
            "session_directory": self.session_dir,
            "camera_model": self.device_model,
            "color_device": self.color_device,
            "depth_device": self.depth_device,
            "total_captures": captured_count,
            "capture_format": "color.jpg + depth.png + metadata.json",
            "next_steps": [
                f"python pointcloud_files/turn_RGBD_to_pointcloud.py {self.session_dir}"
            ],
        }
        path = os.path.join(self.session_dir, "session_summary.json")
        with open(path, 'w') as f:
            json.dump(summary, f, indent=2)
        print(f"Session summary: {path}")

    def run_capture_session(self):
        if not self.setup_zr300_streams():
            return

        print("\nControls:")
        print("  s - save RGB-D triplet")
        print("  q - quit")

        capture_count = 0
        while True:
            color_image, depth_image = self.get_zr300_frames()

            if color_image is not None:
                cv2.imshow(f"ZR300 Color (/dev/video{self.color_device})", color_image)

            if depth_image is not None:
                dvis = self._depth_for_display(depth_image)
                if dvis is not None:
                    cv2.imshow(f"ZR300 Depth (/dev/video{self.depth_device})", dvis)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                if color_image is None or depth_image is None:
                    print("Cannot save: missing color or depth frame")
                else:
                    self.save_rgbd_triplet(color_image, depth_image)
                    capture_count += 1

        self.create_session_summary(capture_count)
        print(f"Captured: {capture_count} triplets")
        print(f"Saved to: {self.session_dir}")
        self.cleanup()

    def cleanup(self):
        if self.color_cap:
            self.color_cap.release()
        if self.depth_cap:
            self.depth_cap.release()
        cv2.destroyAllWindows()
        print("Resources cleaned up")

def main():
    print("=== ZR300 RGB-D CAPTURE (OpenCV/V4L2) ===")
    session = RGBDCaptureSession()
    session.run_capture_session()

if __name__ == "__main__":
    main()