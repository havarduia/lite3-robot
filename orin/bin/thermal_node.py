#!/usr/bin/env python3
"""ROS 2 node for the USB thermal camera: PureThermal board + FLIR Lepton 3.0.

The camera (found 2026-10-15) is a GroupGets PureThermal (fw 1.3.0, USB
1e4e:0100, a standard UVC camera) carrying a Lepton 3.0 (part 500-0726-01):
160 x 120 at 9 fps, NOT radiometric - pixel values are relative heat, not
temperatures (only the Lepton 3.5 can report degrees; its radiometry
controls answer "disabled"/unsupported here). Published, frame
"thermal_camera", ROS_DOMAIN_ID as set (42 on the lite3 Jetson):

    /thermal/image_raw    mono16  raw 14-bit Lepton values (Y16), for processing
    /thermal/image        mono8   auto-contrast (2nd-98th percentile per frame)
    /thermal/image_color  rgb8    the same through a heat colour map, for people
    /thermal/camera_info          approximate: pinhole from the 57 deg field of
                                  view, no distortion - not a calibration
    /dev/shm/thermal_color.jpg    the latest colour frame, for the web panel

    thermal-node                          the camera found by its USB id
    thermal-node --ros-args -p device:=/dev/video0
    thermal-node --ros-args -p rotate:=90         turn the picture 90 deg clockwise

Reconnects by itself if the camera is unplugged or drops off the bus (it
re-enumerated once at boot). The Lepton pauses ~0.5 s every few minutes for
its flat-field correction (a click from the shutter) - that is normal.
"""
import glob
import math
import os
import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

BY_ID = "/dev/v4l/by-id/usb-GroupGets_PureThermal*-video-index0"
W, H = 160, 120
HFOV_DEG = 57.0      # Lepton 3.0 horizontal field of view
JPEG = "/dev/shm/thermal_color.jpg"   # latest colour frame, for the web panel


class Thermal(Node):
    def __init__(self):
        super().__init__("thermal_camera")
        self.device = self.declare_parameter("device", "").value
        self.frame_id = self.declare_parameter("frame_id", "thermal_camera").value
        # The camera is mounted on its side: turn the picture upright here
        # (clockwise degrees: 0, 90, 180 or 270; thermal_start.sh sets it).
        self.rotate = int(self.declare_parameter("rotate", 0).value) % 360
        if self.rotate not in (0, 90, 180, 270):
            raise ValueError("rotate must be 0, 90, 180 or 270")
        self.pub_raw = self.create_publisher(Image, "/thermal/image_raw", 5)
        self.pub_8 = self.create_publisher(Image, "/thermal/image", 5)
        self.pub_col = self.create_publisher(Image, "/thermal/image_color", 5)
        self.pub_info = self.create_publisher(CameraInfo, "/thermal/camera_info", 5)
        self.info = self.camera_info()
        self.cap = None
        self.frames = 0
        self.last_log = time.monotonic()

    def camera_info(self):
        f = (W / 2) / math.tan(math.radians(HFOV_DEG) / 2)   # square pixels
        w, h = (H, W) if self.rotate in (90, 270) else (W, H)
        m = CameraInfo()
        m.header.frame_id = self.frame_id
        m.width, m.height = w, h
        m.distortion_model = "plumb_bob"
        m.d = [0.0] * 5
        m.k = [f, 0.0, w / 2, 0.0, f, h / 2, 0.0, 0.0, 1.0]
        m.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        m.p = [f, 0.0, w / 2, 0.0, 0.0, f, h / 2, 0.0, 0.0, 0.0, 1.0, 0.0]
        return m

    def open(self):
        dev = self.device or next(iter(sorted(glob.glob(BY_ID))), "")
        if not dev:
            return False
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"Y16 "))
        cap.set(cv2.CAP_PROP_CONVERT_RGB, 0)
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, H)      # not 122: no telemetry rows
        if not cap.isOpened():
            cap.release()
            return False
        self.cap = cap
        self.get_logger().info(f"thermal camera open: {dev}")
        return True

    def image(self, stamp, enc, a):
        m = Image()
        m.header.stamp, m.header.frame_id = stamp, self.frame_id
        m.height, m.width = a.shape[:2]
        m.encoding = enc
        m.is_bigendian = 0
        m.step = a.strides[0]
        m.data = a.tobytes()
        return m

    def write_jpeg(self, bgr):
        """The latest colour frame for the web panel's Thermal view: a JPEG in
        RAM (/dev/shm), replaced atomically so a reader never sees half of
        one. A few KB, 9 times a second; nothing on the SD card."""
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 90])
        if not ok:
            return
        tmp = JPEG + ".tmp"
        try:
            with open(tmp, "wb") as f:
                f.write(buf.tobytes())
            os.replace(tmp, JPEG)
        except OSError:
            pass

    def step(self):
        if self.cap is None:
            if not self.open():
                time.sleep(2.0)
                return
        ok, f = self.cap.read()
        if not ok or f is None:
            self.get_logger().warn("no frame - reopening the camera")
            self.cap.release()
            self.cap = None
            time.sleep(1.0)
            return
        raw = f.view(np.uint16).reshape(-1, W)[:H]
        if self.rotate:
            raw = np.rot90(raw, -self.rotate // 90)     # negative k: clockwise
        raw = np.ascontiguousarray(raw)
        stamp = self.get_clock().now().to_msg()
        self.pub_raw.publish(self.image(stamp, "mono16", raw))
        lo, hi = np.percentile(raw, (2, 98))
        g = np.clip((raw.astype(np.float32) - lo) * (255.0 / max(hi - lo, 1.0)), 0, 255).astype(np.uint8)
        bgr = cv2.applyColorMap(g, cv2.COLORMAP_INFERNO)
        if self.pub_8.get_subscription_count():
            self.pub_8.publish(self.image(stamp, "mono8", g))
        if self.pub_col.get_subscription_count():
            self.pub_col.publish(self.image(stamp, "rgb8", cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)))
        self.write_jpeg(bgr)
        self.info.header.stamp = stamp
        self.pub_info.publish(self.info)
        self.frames += 1
        if time.monotonic() - self.last_log > 60:
            self.get_logger().info(f"{self.frames} frames published")
            self.last_log = time.monotonic()


def main():
    rclpy.init()
    node = Thermal()
    try:
        while rclpy.ok():
            node.step()                     # paced by the camera (9 fps)
            rclpy.spin_once(node, timeout_sec=0)
    except KeyboardInterrupt:
        pass
    finally:
        if node.cap is not None:
            node.cap.release()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
