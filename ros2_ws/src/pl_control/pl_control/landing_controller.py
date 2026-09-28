import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from vision_msgs.msg import Detection3DArray
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLandDetected,
    VehicleLocalPosition,
    VehicleStatus,
)

TRACK, BLIND, HOLD, DONE = "TRACK", "BLIND", "HOLD", "DONE"


class LandingController(Node):
    """Yaw-invariant precision landing.

    TRACK : tag visible - centering + vision altitude ramp
    BLIND : tag lost on final - constant slow sink, motion-based contact
    HOLD  : blind budget exhausted - float and complain
    DONE  : contact - cut thrust via offboard actuators so PX4's own land
            detector fires, then normal disarm; force only as last resort
    """

    def __init__(self):
        super().__init__("landing_controller")
        self.declare_parameter("allow_descent", True)
        self.k_xy = 0.5
        self.v_max = 0.8
        self.k_z = 0.8
        self.vz_max = 0.5
        self.ramp_per_tick = 0.0025
        self.touchdown = 0.25
        self.blind_entry = 0.6
        self.blind_rate = 0.12
        self.blind_budget = 25.0

        self.state = TRACK
        self.oz_target = None
        self.last_oz = None
        self.det = None
        self.det_time = self.get_clock().now()
        self.heading = 0.0
        self.landed = False
        self.armed = True
        self.local_z = None
        self.probe_time = None
        self.probe_z = None
        self.stall_count = 0
        self.blind_start = None
        self.done_ticks = 0
        self.disarm_ticks = 0
        self.announced = False
        self.last_log_sec = -1

        qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.sub = self.create_subscription(
            Detection3DArray, "/pl/perception/tags", self.det_cb, 10)
        self.sub_pos = self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position",
            self.pos_cb, qos)
        self.sub_land = self.create_subscription(
            VehicleLandDetected, "/fmu/out/vehicle_land_detected",
            self.land_cb, qos)
        self.sub_status = self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.status_cb, qos)
        self.pub_mode = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos)
        self.timer = self.create_timer(0.05, self.cmd_cb)

    def pos_cb(self, msg):
        self.heading = msg.heading
        self.local_z = msg.z          # NED: down positive

    def land_cb(self, msg):
        self.landed = bool(msg.landed or msg.ground_contact)

    def status_cb(self, msg):
        self.armed = (msg.arming_state == 2)   # 2 == ARMING_STATE_ARMED

    def det_cb(self, msg):
        if msg.detections and msg.detections[0].results:
            p = msg.detections[0].results[0].pose.pose.position
            self.det = (p.x, p.y, p.z)
            self.det_time = self.get_clock().now()

    def _disarm(self, ts, force=False):
        cmd = VehicleCommand()
        cmd.timestamp = ts
        cmd.command = 400          # VEHICLE_CMD_ARM_DISARM
        cmd.param1 = 0.0           # disarm
        cmd.param2 = 211930.0 if force else 0.0
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.from_external = True
        self.pub_cmd.publish(cmd)

    def cmd_cb(self):
        now = self.get_clock().now()
        ts = int(now.nanoseconds // 1000)

        stale = (now - self.det_time).nanoseconds * 1e-9 > 0.5
        fresh = self.det is not None and not stale

        if self.state == DONE:
            self.done_ticks += 1
            # Hand the final meters to PX4's own AUTO_LAND: it owns the
            # ground detection and auto-disarm logic.
            if self.done_ticks <= 10 and self.done_ticks % 2 == 0:
                cmd = VehicleCommand()
                cmd.timestamp = ts
                cmd.command = 21  # VEHICLE_CMD_NAV_LAND
                cmd.param1 = 0.0
                cmd.target_system = 1
                cmd.target_component = 1
                cmd.from_external = True
                self.pub_cmd.publish(cmd)
            # Publish no offboard setpoints: if NAV_LAND is ever refused,
            # offboard-loss failsafe lands and disarms as a backstop.
            if not self.armed and not self.announced:
                self.announced = True
                self.get_logger().info(
                    "state=DONE disarmed by PX4, mission complete")
            return

        mode = OffboardControlMode()
        mode.timestamp = ts
        mode.velocity = True
        self.pub_mode.publish(mode)

        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position = [float("nan")] * 3
        vx = vy = vz = 0.0
        opt = None

        if self.state == TRACK:
            if fresh:
                ox, oy, oz = self.det
                opt = (ox, oy, oz)
                self.last_oz = oz
                bx, by = -oy, -ox
                v_bx = max(-self.v_max, min(self.v_max, self.k_xy * bx))
                v_by = max(-self.v_max, min(self.v_max, self.k_xy * -by))
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
                    vz = max(-self.vz_max, min(
                        self.vz_max, self.k_z * (oz - self.oz_target)))
                    if oz <= self.touchdown + 0.02:
                        vz = 0.0
                else:
                    self.oz_target = None
            else:
                if (self.get_parameter("allow_descent").value
                        and self.last_oz is not None
                        and self.last_oz <= self.blind_entry):
                    self.state = BLIND
                    self.blind_start = now
                    self.probe_time = None
                    self.probe_z = None
                    self.stall_count = 0
                    self.get_logger().warn(
                        f"tag lost at {self.last_oz:.2f} m: entering BLIND descent")

        elif self.state == BLIND:
            vx = vy = 0.0
            vz = self.blind_rate
            elapsed = (now - self.blind_start).nanoseconds * 1e-9

            if self.probe_time is None:
                self.probe_time = now
                self.probe_z = self.local_z
            else:
                dt = (now - self.probe_time).nanoseconds * 1e-9
                if dt >= 1.0:
                    if self.local_z is not None and self.probe_z is not None:
                        if (self.local_z - self.probe_z) < 0.01:
                            self.stall_count += 1
                        else:
                            self.stall_count = 0
                    self.probe_time = now
                    self.probe_z = self.local_z

            if self.landed or self.stall_count >= 2:
                self.state = DONE
                self.get_logger().warn(
                    "CONTACT detected: handing touchdown to PX4 AUTO_LAND")
            elif elapsed > self.blind_budget:
                self.state = HOLD
                self.get_logger().error(
                    "BLIND budget exhausted without contact: HOLDING")

        elif self.state == HOLD:
            vx = vy = vz = 0.0

        sp.velocity = [vx, vy, vz]
        self.pub_sp.publish(sp)

        if now.nanoseconds // 1_000_000_000 != self.last_log_sec:
            self.last_log_sec = now.nanoseconds // 1_000_000_000
            tgt = "--.--" if self.oz_target is None else f"{self.oz_target:.2f}"
            if opt is None:
                self.get_logger().info(
                    f"state={self.state} opt=NONE tgt={tgt} "
                    f"cmd=(0.00, 0.00, {vz:+.2f}) stall={self.stall_count}")
            else:
                self.get_logger().info(
                    f"state={self.state} opt=({opt[0]:+.2f},{opt[1]:+.2f},"
                    f"{opt[2]:.2f}) tgt={tgt} yaw={math.degrees(self.heading):+.0f} "
                    f"cmd_ned=({vx:+.2f},{vy:+.2f},{vz:+.2f})")


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
