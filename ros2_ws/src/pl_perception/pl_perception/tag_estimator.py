"""Exponential Moving Average (EMA) filter over tag pose in the optical frame.

A 6-state Constant Velocity KF fails in the optical frame because the drone's
flight controller actively damps velocity, violating the CV assumption and
causing phase lag. An EMA is a 1st-order low-pass filter: it cannot diverge,
has no velocity states to fight the drone, and introduces minimal, predictable lag.
"""
import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3D, Detection3DArray, ObjectHypothesisWithPose


class TagEstimator(Node):
    def __init__(self):
        super().__init__("tag_estimator")
        self.alpha = 0.2  # 0.0 = pure prediction (lag), 1.0 = pure raw (jitter)
        self.max_age = 0.6
        self.last_meas_time = None
        self.x_est = None
        self.last_log = -1

        self.create_subscription(
            Detection3DArray, "/pl/perception/tags", self.det_cb, 10)
        self.pub = self.create_publisher(
            Detection3DArray, "/pl/perception/tags_est", 10)
        self.timer = self.create_timer(0.05, self.tick)

    def det_cb(self, msg):
        if not msg.detections or not msg.detections[0].results:
            return
        p = msg.detections[0].results[0].pose.pose.position
        z = (p.x, p.y, p.z)
        now = self.get_clock().now()
        self.last_meas_time = now
        
        if self.x_est is None:
            self.x_est = z
        else:
            self.x_est = (
                self.alpha * z[0] + (1 - self.alpha) * self.x_est[0],
                self.alpha * z[1] + (1 - self.alpha) * self.x_est[1],
                self.alpha * z[2] + (1 - self.alpha) * self.x_est[2],
            )

    def tick(self):
        now = self.get_clock().now()
        out = Detection3DArray()
        out.header.stamp = now.to_msg()
        
        age = 9.9
        if self.last_meas_time is not None:
            age = (now - self.last_meas_time).nanoseconds * 1e-9
            
        if self.x_est is not None and age < self.max_age:
            det = Detection3D()
            hyp = ObjectHypothesisWithPose()
            hyp.hypothesis.class_id = "0"
            hyp.hypothesis.score = 1.0
            hyp.pose.pose.position.x = float(self.x_est[0])
            hyp.pose.pose.position.y = float(self.x_est[1])
            hyp.pose.pose.position.z = float(self.x_est[2])
            hyp.pose.pose.orientation.w = 1.0
            det.results.append(hyp)
            out.detections.append(det)
        self.pub.publish(out)

        sec = now.nanoseconds // 1_000_000_000
        if sec != self.last_log:
            self.last_log = sec
            self.get_logger().info(f"est age={age:.2f}s alpha={self.alpha}")


def main(args=None):
    rclpy.init(args=args)
    node = TagEstimator()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
