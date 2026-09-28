import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from vision_msgs.msg import Detection3DArray
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleLocalPosition


class LandingController(Node):
    """Yaw-invariant precision landing controller."""

    def __init__(self):
        super().__init__("landing_controller")
        self.declare_parameter("allow_descent", True)
        self.k_xy = 0.5
        self.v_max = 0.8
        self.k_z = 0.8
        self.vz_max = 0.5
        self.ramp_per_tick = 0.0025   
        self.touchdown = 0.25         
        
        self.oz_target = None
        self.det = None
        self.det_time = self.get_clock().now()
        self.heading = 0.0  # Current yaw in radians (NED)
        self.last_log_sec = -1

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        
        self.sub = self.create_subscription(
            Detection3DArray, "/pl/perception/tags", self.det_cb, 10)
            
        # NEW: Subscribe to PX4 local position to get the current heading
        self.sub_pos = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position", self.pos_cb, qos)
            
        self.pub_mode = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self.timer = self.create_timer(0.05, self.cmd_cb)

    def pos_cb(self, msg):
        # Update our knowledge of the drone's current yaw
        self.heading = msg.heading

    def det_cb(self, msg):
        if msg.detections and msg.detections[0].results:
            p = msg.detections[0].results[0].pose.pose.position
            self.det = (p.x, p.y, p.z)
            self.det_time = self.get_clock().now()

    def cmd_cb(self):
        now = self.get_clock().now()
        ts = int(now.nanoseconds // 1000)

        mode = OffboardControlMode()
        mode.timestamp = ts
        mode.velocity = True
        self.pub_mode.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position = [float("nan")] * 3
        vx = vy = vz = 0.0
        opt = None

        stale = (now - self.det_time).nanoseconds * 1e-9 > 0.5
        if self.det is not None and not stale:
            ox, oy, oz = self.det
            opt = (ox, oy, oz)
            bx, by = -oy, -ox          # Measured optical->body mapping
            
            # 1. Calculate desired velocity in the BODY frame
            v_bx = max(-self.v_max, min(self.v_max, self.k_xy * bx))
            v_by = max(-self.v_max, min(self.v_max, self.k_xy * -by))
            
            # 2. Rotate body velocity into the NED frame using current heading
            cy = math.cos(self.heading)
            sy = math.sin(self.heading)
            vx = v_bx * cy - v_by * sy
            vy = v_bx * sy + v_by * cy

            h_err = math.hypot(bx, by)
            if h_err < 0.15:
                if self.oz_target is None:
                    self.oz_target = oz
                if self.get_parameter("allow_descent").value:
                    self.oz_target = max(
                        self.oz_target - self.ramp_per_tick, self.touchdown)
                vz = max(-self.vz_max, min(self.vz_max,
                                           self.k_z * (oz - self.oz_target)))
                if oz <= self.touchdown + 0.02:
                    vz = 0.0
            else:
                self.oz_target = None
        else:
            self.oz_target = None

        sp.velocity = [vx, vy, vz]
        self.pub_sp.publish(sp)

        if now.nanoseconds // 1_000_000_000 != self.last_log_sec:
            self.last_log_sec = now.nanoseconds // 1_000_000_000
            tgt = "  --  " if self.oz_target is None else f"{self.oz_target:.2f}"
            if opt is None:
                self.get_logger().info("opt=NONE tgt=--.-- cmd=(0.00, 0.00, 0.00) HOLD")
            else:
                # Log the final NED commands being sent
                self.get_logger().info(
                    f"opt=({opt[0]:+.2f},{opt[1]:+.2f},{opt[2]:.2f}) tgt={tgt} "
                    f"yaw={math.degrees(self.heading):+.0f} cmd_ned=({vx:+.2f},{vy:+.2f},{vz:+.2f})")


def main(args=None):
    rclpy.init(args=args)
    node = LandingController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
