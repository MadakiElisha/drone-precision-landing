import os

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image

OUT = "dataset/images"


class Recorder(Node):
    def __init__(self):
        super().__init__("dataset_recorder")
        os.makedirs(OUT, exist_ok=True)
        self.count = 0
        self.saved = 0
        self.k_done = False
        self.create_subscription(Image, "/pl/camera/image_raw", self.img_cb, 10)
        self.create_subscription(CameraInfo, "/pl/camera/camera_info", self.k_cb, 10)

    def k_cb(self, msg):
        if not self.k_done:
            with open("dataset/K.txt", "w") as f:
                f.write(" ".join(f"{v:.6f}" for v in msg.k))
            self.k_done = True

    def img_cb(self, msg):
        self.count += 1
        if self.count % 6 != 0:      # ~5 Hz at 30 fps sim camera
            return
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        rgb = buf.reshape(msg.height, msg.width, 3)
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
        t = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        cv2.imwrite(f"{OUT}/{t}.png", bgr)
        self.saved += 1
        if self.saved % 50 == 0:
            self.get_logger().info(f"saved {self.saved} frames")


def main():
    rclpy.init()
    node = Recorder()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
