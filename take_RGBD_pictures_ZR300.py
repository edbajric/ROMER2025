import cv2
import numpy as np
import os
import json
import sys
import subprocess
from datetime import datetime

class ZR300Capture:
    def __init__(self, color_id=None, depth_id=None, width=640, height=480, fps=30):
        self.color_id = color_id
        self.depth_id = depth_id
        self.width = width
        self.height = height
        self.fps = fps

        self.color_cap = None
        self.depth_cap = None

        self.session_dir = None
        self._create_session_dir()

    def _create_session_dir(self):
        base = "RGBD_photos"
        os.makedirs(base, exist_ok=True)
        ts = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.session_dir = os.path.join(base, f"session_{ts}")
        os.makedirs(self.session_dir, exist_ok=True)
        print(f"Session: {self.session_dir}")

    def list_devices(self, max_dev=16):
        print("Scanning /dev/video* for ZR300 streams...")
        found = []
        for i in range(max_dev):
            dev = f"/dev/video{i}"
            if not os.path.exists(dev):
                continue
            info = self._v4l2_info(i)
            fmts = self._v4l2_formats(i)
            role = self._guess_role_with_formats(i, fmts)
            if role == "unknown":
                role2, tags = self._opencv_probe_role(i)
                role = role2
                fmts = (fmts or []) + tags
            print(f"- {dev}: role={role} formats={', '.join(fmts) if fmts else 'unknown'} info={info}")
            found.append((i, role, fmts))
        return found

    def _v4l2_info(self, idx):
        try:
            r = subprocess.run(
                ["v4l2-ctl", "-d", f"/dev/video{idx}", "--info"],
                capture_output=True, text=True, timeout=2
            )
            txt = r.stdout.strip()
            return "ZR300" if ("RealSense" in txt or "Intel" in txt) else "generic"
        except Exception:
            return "unknown"

    def _v4l2_formats(self, idx):
        try:
            r = subprocess.run(
                ["v4l2-ctl", "-d", f"/dev/video{idx}", "--list-formats-ext"],
                capture_output=True, text=True, timeout=2
            )
            fmts = []
            for line in r.stdout.splitlines():
                line = line.strip()
                if line.startswith("Pixel Format") or line.startswith("Type"):
                    fmts.append(line)
            tokens = []
            for f in fmts:
                if "YUYV" in f: tokens.append("YUYV")
                if "MJPG" in f or "MJPEG" in f: tokens.append("MJPG")
                if "RGB" in f and "BGR" not in f: tokens.append("RGB")
                if "GREY" in f or "GRAY" in f: tokens.append("GREY")
                if "Y16" in f: tokens.append("Y16")
            return list(dict.fromkeys(tokens))
        except Exception:
            return []

    def _guess_role_with_formats(self, idx, formats):
        # Heuristic: Color likely YUYV/MJPG/RGB, Depth likely Y16/GREY
        if "Y16" in formats:
            return "depth"
        if "GREY" in formats and not {"YUYV","MJPG","RGB"}.intersection(formats):
            return "depth"
        if {"YUYV","MJPG","RGB"}.intersection(formats):
            return "color"
        return "unknown"

    def _open_stream(self, idx, force_fourcc=None):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            return None
        if force_fourcc:
            cap.set(cv2.CAP_PROP_FOURCC, force_fourcc)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        # Small warmup
        for _ in range(5):
            cap.read()
        return cap if cap.isOpened() else None

    def _opencv_probe_role(self, idx, tries=6):
        cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
        if not cap.isOpened():
            return "unknown", []
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        cap.set(cv2.CAP_PROP_FPS, self.fps)
        role = "unknown"
        ch = None
        for _ in range(tries):
            ret, frm = cap.read()
            if not ret or frm is None:
                continue
            if len(frm.shape) == 2:
                role, ch = "depth", 1
                break
            if len(frm.shape) == 3:
                if frm.shape[2] == 2:
                    role, ch = "color", 2
                    break
                if frm.shape[2] == 3:
                    role, ch = "color", 3
                    break
        cap.release()
        tag = f"opencv_ch{ch}" if ch else "opencv_no_frame"
        return role, [tag]

    def setup_streams(self):
        if self.color_id is None or self.depth_id is None:
            devices = self.list_devices()
            if self.color_id is None:
                cand = [i for i, role, _ in devices if role == "color"]
                if cand:
                    self.color_id = cand[0]
            if self.depth_id is None:
                cand = [i for i, role, _ in devices if role == "depth" and i != self.color_id]
                if cand:
                    self.depth_id = cand[0]
            # Final fallback: probe every node
            if self.color_id is None:
                for i in range(16):
                    if not os.path.exists(f"/dev/video{i}"):
                        continue
                    role, _ = self._opencv_probe_role(i)
                    if role == "color":
                        self.color_id = i
                        break
            if self.depth_id is None:
                for i in range(16):
                    if not os.path.exists(f"/dev/video{i}") or i == self.color_id:
                        continue
                    role, _ = self._opencv_probe_role(i)
                    if role == "depth":
                        self.depth_id = i
                        break

        if self.color_id is None and self.depth_id is None:
            print("No suitable ZR300 devices found.")
            return False

        # Try multiple FOURCCs for robustness
        if self.color_id is not None:
            for fourcc in [cv2.VideoWriter_fourcc('M','J','P','G'),
                           cv2.VideoWriter_fourcc('Y','U','Y','V'),
                           0]:
                self.color_cap = self._open_stream(self.color_id, force_fourcc=fourcc if fourcc != 0 else None)
                if self.color_cap:
                    print(f"Color stream: /dev/video{self.color_id} (fourcc={'MJPG' if fourcc and fourcc!=0 else 'default' if fourcc==0 else 'YUYV'})")
                    break

        if self.depth_id is not None:
            for fourcc in [cv2.VideoWriter_fourcc('Y','1','6',' '),
                           cv2.VideoWriter_fourcc('G','R','E','Y'),
                           0]:
                self.depth_cap = self._open_stream(self.depth_id, force_fourcc=fourcc if fourcc != 0 else None)
                if self.depth_cap:
                    print(f"Depth stream: /dev/video{self.depth_id} (fourcc={'Y16' if fourcc and fourcc!=0 else 'default' if fourcc==0 else 'GREY'})")
                    break

        ok = (self.color_cap is not None) or (self.depth_cap is not None)
        if not ok:
            print("Failed to open color/depth streams.")
        else:
            print("Streams initialized.")
            print(f"  Color: {self.color_id if self.color_cap else 'N/A'}")
            print(f"  Depth: {self.depth_id if self.depth_cap else 'N/A'}")
        return ok

    def read_frames(self):
        color = None
        depth = None

        if self.color_cap:
            ret, frm = self.color_cap.read()
            if ret and frm is not None:
                # Convert YUYV->BGR only if 2-channel image is received
                if len(frm.shape) == 3 and frm.shape[2] == 2:
                    color = cv2.cvtColor(frm, cv2.COLOR_YUV2BGR_YUYV)
                elif len(frm.shape) == 3 and frm.shape[2] == 3:
                    color = frm
                elif len(frm.shape) == 2:
                    # Gray fallback; convert to BGR for consistency
                    color = cv2.cvtColor(frm, cv2.COLOR_GRAY2BGR)

        if self.depth_cap:
            ret, frm = self.depth_cap.read()
            if ret and frm is not None:
                # If 3-channel, convert to grayscale
                if len(frm.shape) == 3 and frm.shape[2] == 3:
                    depth = cv2.cvtColor(frm, cv2.COLOR_BGR2GRAY)
                else:
                    depth = frm
                # Try to preserve 16-bit if available; otherwise stay uint8
                if depth.dtype != np.uint16 and len(depth.shape) == 2 and depth.max() > 255:
                    depth = depth.astype(np.uint16)

        return color, depth

    def _depth_for_display(self, depth):
        if depth is None:
            return None
        if depth.dtype == np.uint16:
            # Normalize 16-bit to 8-bit for display
            d8 = cv2.convertScaleAbs(depth, alpha=255.0 / max(1, depth.max()))
        else:
            d8 = depth if depth.dtype == np.uint8 else cv2.convertScaleAbs(depth)
        return cv2.applyColorMap(d8, cv2.COLORMAP_JET)

    def save_triplet(self, color, depth):
        ts = datetime.now().strftime("%Y%m%d_%H%M%S_%f")[:-3]
        color_name = f"color_{ts}.jpg" if color is not None else None
        depth_name = f"depth_{ts}.png" if depth is not None else None
        meta_name = f"rgbd_{ts}.json"

        if color is not None:
            cv2.imwrite(os.path.join(self.session_dir, color_name), color)
        if depth is not None:
            # Save depth as PNG; preserves uint16 if provided
            cv2.imwrite(os.path.join(self.session_dir, depth_name), depth)

        meta = {
            "timestamp": ts,
            "session_dir": self.session_dir,
            "color_image": color_name,
            "depth_image": depth_name,
            "color_device": self.color_id,
            "depth_device": self.depth_id,
            "resolution": [self.width, self.height],
            "fps": self.fps,
            "color_format": "BGR",
            "depth_format": "Y16_or_GREY_png",
        }
        with open(os.path.join(self.session_dir, meta_name), "w") as f:
            json.dump(meta, f, indent=2)

        print(f"Saved triplet: {ts}")

    def run(self):
        if not self.setup_streams():
            print("Initialization failed.")
            return

        print("\nControls:")
        print("  s - save RGB-D triplet")
        print("  q - quit")

        saved = 0
        try:
            while True:
                color, depth = self.read_frames()

                if color is not None:
                    cv2.imshow(f"ZR300 Color (/dev/video{self.color_id})", color)
                if depth is not None:
                    dvis = self._depth_for_display(depth)
                    if dvis is not None:
                        cv2.imshow(f"ZR300 Depth (/dev/video{self.depth_id})", dvis)

                key = cv2.waitKey(1) & 0xFF
                if key == ord('q'):
                    break
                elif key == ord('s'):
                    if color is None or depth is None:
                        print("Missing color or depth frame; not saved.")
                    else:
                        self.save_triplet(color, depth)
                        saved += 1
        finally:
            self._summary(saved)
            self.cleanup()

    def _summary(self, count):
        summary = {
            "session_timestamp": datetime.now().strftime("%Y%m%d_%H%M%S"),
            "session_directory": self.session_dir,
            "color_device": self.color_id,
            "depth_device": self.depth_id,
            "total_captures": count,
            "capture_format": "color.jpg + depth.png + metadata.json",
            "next_steps": [
                f"python pointcloud_files/turn_RGBD_to_pointcloud.py {self.session_dir}"
            ],
        }
        with open(os.path.join(self.session_dir, "session_summary.json"), "w") as f:
            json.dump(summary, f, indent=2)
        print(f"Session summary: {os.path.join(self.session_dir, 'session_summary.json')}")

    def cleanup(self):
        if self.color_cap:
            self.color_cap.release()
        if self.depth_cap:
            self.depth_cap.release()
        cv2.destroyAllWindows()
        print("Resources released.")

def parse_args():
    import argparse
    p = argparse.ArgumentParser(description="ZR300 RGB-D capture via OpenCV/V4L2")
    p.add_argument("--color", type=str, default=None, help="color device index or path (e.g., 2 or /dev/video2)")
    p.add_argument("--depth", type=str, default=None, help="depth device index or path (e.g., 4 or /dev/video4)")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--list", action="store_true", help="list devices and exit")
    return p.parse_args()

def to_index(val):
    if val is None:
        return None
    if isinstance(val, int):
        return val
    s = str(val)
    if s.startswith("/dev/video"):
        try:
            return int(s.replace("/dev/video", ""))
        except ValueError:
            return None
    try:
        return int(s)
    except ValueError:
        return None

def main():
    args = parse_args()

    cidx = to_index(args.color)
    didx = to_index(args.depth)

    cap = ZR300Capture(color_id=cidx, depth_id=didx, width=args.width, height=args.height, fps=args.fps)
    if args.list:
        cap.list_devices()
        return
    cap.run()

if __name__ == "__main__":
    main()