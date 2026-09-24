import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
import cv2
import numpy as np


class ArucoDetector(Node):
    def __init__(self):
        super().__init__("aruco_detector")
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        self.declare_parameter("marker_size", 0.4)
        self.marker_size = self.get_parameter("marker_size").value

        self.K = None

        self.sub_info = self.create_subscription(
            CameraInfo, "/pl/camera/camera_info", self.info_cb, 10)
        self.sub_img = self.create_subscription(
            Image, "/pl/camera/image_raw", self.img_cb, 10)
        self.pub = self.create_publisher(Detection3DArray, "/pl/perception/tags", 10)

    def info_cb(self, msg):
        self.K = np.array(msg.k, dtype=np.float64).reshape(3, 3)

    def _to_gray(self, msg):
        """Convert ROS Image to grayscale numpy without cv_bridge."""
        buf = np.frombuffer(msg.data, dtype=np.uint8)
        if msg.encoding == "mono8":
            return buf.reshape(msg.height, msg.width)
        if msg.encoding == "rgb8":
            return cv2.cvtColor(buf.reshape(msg.height, msg.width, 3), cv2.COLOR_RGB2GRAY)
        if msg.encoding == "bgr8":
            return cv2.cvtColor(buf.reshape(msg.height, msg.width, 3), cv2.COLOR_BGR2GRAY)
        self.get_logger().warn(f"unsupported encoding: {msg.encoding}", once=True)
        return None

    def img_cb(self, msg):
        if self.K is None:
            return
        gray = self._to_gray(msg)
        if gray is None:
            return

        corners, ids, _ = self.detector.detectMarkers(gray)

        out = Detection3DArray()
        out.header = msg.header

        if ids is not None:
            half = self.marker_size / 2.0
            obj_pts = np.array([
                [-half,  half, 0.0],
                [ half,  half, 0.0],
                [ half, -half, 0.0],
                [-half, -half, 0.0],
            ], dtype=np.float32)

            for corner, tag_id in zip(corners, ids.flatten()):
                ok, rvec, tvec = cv2.solvePnP(
                    obj_pts, corner.reshape(4, 2), self.K, None)
                if not ok:
                    continue
                tvec = tvec.flatten()
                R, _ = cv2.Rodrigues(rvec)
                qw = np.sqrt(max(1.0 + R[0, 0] + R[1, 1] + R[2, 2], 1e-6)) / 2.0
                qx = (R[2, 1] - R[1, 2]) / (4.0 * qw)
                qy = (R[0, 2] - R[2, 0]) / (4.0 * qw)
                qz = (R[1, 0] - R[0, 1]) / (4.0 * qw)

                det = Detection3D()
                hyp = ObjectHypothesisWithPose()
                hyp.hypothesis.class_id = str(int(tag_id))
                hyp.hypothesis.score = 1.0
                hyp.pose.pose.position.x = float(tvec[0])
                hyp.pose.pose.position.y = float(tvec[1])
                hyp.pose.pose.position.z = float(tvec[2])
                hyp.pose.pose.orientation.x = float(qx)
                hyp.pose.pose.orientation.y = float(qy)
                hyp.pose.pose.orientation.z = float(qz)
                hyp.pose.pose.orientation.w = float(qw)
                det.results.append(hyp)

                pts = corner.reshape(4, 2)
                det.bbox.center.position.x = float(pts[:, 0].mean())
                det.bbox.center.position.y = float(pts[:, 1].mean())
                det.bbox.size.x = float(np.ptp(pts[:, 0]))
                det.bbox.size.y = float(np.ptp(pts[:, 1]))
                out.detections.append(det)

        self.pub.publish(out)


def main(args=None):
    rclpy.init(args=args)
    node = ArucoDetector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
