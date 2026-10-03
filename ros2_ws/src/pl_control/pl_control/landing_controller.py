"""Landing controller v5 — clean rewrite.

States: WAYPOINT (optional circuit) -> CLIMB -> TRACK -> {BLIND|HOLD} -> DONE.
Consumes filtered optical tag pose; emits offboard velocity setpoints.
Single velocity assignment point; descent ramp always armed in TRACK;
handoff to PX4 NAV_LAND on close-range lock or physical stall.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import (OffboardControlMode, TrajectorySetpoint,
                          VehicleCommand, VehicleLocalPosition,
                          VehicleStatus)
from vision_msgs.msg import Detection3DArray

WAYPOINT, CLIMB, TRACK, BLIND, HOLD, DONE = (
    "WAYPOINT", "CLIMB", "TRACK", "BLIND", "HOLD", "DONE")


class LandingController(Node):
    def __init__(self):
        super().__init__("landing_controller")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)

        self.declare_parameter("tags_topic", "/pl/perception/tags_est")
        self.declare_parameter("allow_descent", True)
        self.declare_parameter("auto_climb", True)
        self.declare_parameter("waypoints", "")
        self.declare_parameter("spawn", "0,0")

        # gains
        self.k_xy = 0.5
        self.k_z = 0.8
        self.vxy_max = 1.0
        self.vz_max = 0.8
        self.touchdown = 0.25
        self.ramp = 0.004
        self.blind_entry = 0.6
        self.sink = 0.12
        self.handoff_oz = 0.55
        self.handoff_h = 0.15
        self.center_gate = 0.35

        # perception / state estimate
        self.opt = None
        self.det_time = None
        self.last_oz = None
        self.local_x = self.local_y = self.local_z = None
        self.heading = 0.0

        # machine
        self.waypoints = []
        for chunk in self.get_parameter("waypoints").value.split(";"):
            if chunk.strip():
                self.waypoints.append(tuple(float(v) for v in chunk.split(",")))
        sx, sy = (float(v) for v in self.get_parameter("spawn").value.split(","))
        self.waypoints = [(wx - sx, wy - sy, a) for (wx, wy, a) in self.waypoints]
        self.wp_idx = 0
        self.state = CLIMB if self.get_parameter("auto_climb").value else (
            WAYPOINT if self.waypoints else TRACK)
        self.oz_target = None
        self.done_ticks = 0
        self.probe_time = None
        self.probe_z = None
        self.stall = 0
        self.centered_descent = False
        self.z_hold = None
        self.stale_since = None
        self.reacq = 0
        self.nav = 0
        self.gate = self.waypoints[-1] if self.waypoints else None
        self.last_log = -1

        topic = self.get_parameter("tags_topic").value
        self.get_logger().info(f"Subscribing to tags topic: {topic}")
        self.create_subscription(Detection3DArray, topic, self.det_cb, 10)
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position",
            self.pos_cb, qos)
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status", self.nav_cb, qos)
        self.pub_mode = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos)
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos)
        self.create_timer(0.05, self.cmd_cb)
        if self.waypoints:
            self.get_logger().info(
                f"circuit: {len(self.waypoints)} local waypoints (spawn={sx},{sy})")

    def det_cb(self, msg):
        if not msg.detections or not msg.detections[0].results:
            return
        p = msg.detections[0].results[0].pose.pose.position
        self.opt = (p.x, p.y, p.z)
        self.last_oz = p.z
        self.det_time = self.get_clock().now()

    def pos_cb(self, msg):
        if not math.isfinite(msg.heading):
            return
        self.local_x, self.local_y, self.local_z = msg.x, msg.y, msg.z
        self.heading = msg.heading

    def nav_cb(self, msg):
        self.nav = msg.nav_state

    def _stall(self, now, active=True):
        if not active:
            self.stall = 0
            self.probe_time, self.probe_z = now, self.local_z
            return
        if self.local_z is None:
            return
        if self.probe_time is None:
            self.probe_time, self.probe_z = now, self.local_z
            return
        dt = (now - self.probe_time).nanoseconds * 1e-9
        if dt >= 1.0:
            if (self.local_z - self.probe_z) < 0.01:
                self.stall += 1
            else:
                self.stall = 0
            self.probe_time, self.probe_z = now, self.local_z

    def _handoff(self, why):
        self.state = DONE
        self.done_ticks = 0
        self.get_logger().warn(f"handoff ({why}): NAV_LAND")

    def cmd_cb(self):
        now = self.get_clock().now()
        ts = int(now.nanoseconds // 1000)
        fresh = (self.opt is not None and self.det_time is not None
                 and (now - self.det_time).nanoseconds * 1e-9 < 0.5)
        vx = vy = vz = 0.0

        if self.state == WAYPOINT:
            wx, wy, walt = self.waypoints[self.wp_idx]
            tz = 0.3 - walt
            if self.local_z is not None:
                dx, dy, dz = wx - self.local_x, wy - self.local_y, tz - self.local_z
                dist = math.sqrt(dx * dx + dy * dy + dz * dz)
                if dist < 0.5:
                    self.wp_idx += 1
                    self.get_logger().info(
                        f"waypoint {self.wp_idx}/{len(self.waypoints)} reached")
                    if self.wp_idx >= len(self.waypoints):
                        self.state = TRACK
                        self.oz_target = None
                        self.get_logger().info("circuit complete: entering TRACK")
                else:
                    sp = min(2.0, 0.8 * dist)
                    vx, vy, vz = sp * dx / dist, sp * dy / dist, sp * dz / dist
                    vz = max(-1.0, min(1.0, vz))

        elif self.state == CLIMB:
            if self.local_z is None:
                pass
            elif self.local_z > -2.2:
                vz = -0.6
            elif self.waypoints:
                self.state = WAYPOINT
                self.get_logger().info("climb complete: entering WAYPOINT")
            else:
                self.state = TRACK
                self.get_logger().info("climb complete: entering TRACK")

        elif self.state == TRACK:
            if fresh:
                self.stale_since = None
                self.z_hold = None
                ox, oy, oz = self.opt
                th = self.heading or 0.0
                c, s = math.cos(th), math.sin(th)
                bx, by = -self.k_xy * oy, self.k_xy * ox
                vx = c * bx - s * by
                vy = s * bx + c * by
                h = math.hypot(vx, vy)
                if h > self.vxy_max:
                    vx, vy = vx / h * self.vxy_max, vy / h * self.vxy_max
                herr = math.hypot(ox, oy)
                if self.oz_target is None:
                    self.oz_target = min(oz, 3.0)
                if self.get_parameter("allow_descent").value:
                    self.oz_target = max(self.oz_target - self.ramp, self.touchdown)
                if herr < self.center_gate:
                    vz = max(0.0, min(self.vz_max, self.k_z * (oz - self.oz_target)))
                    if vz > 0.05:
                        self.centered_descent = True
                self._stall(now, active=(vz > 0.05))
                if ((herr < 0.45 and oz <= self.handoff_oz)
                        or (self.stall >= 2 and oz < 2.0)):
                    self._handoff(f"oz={oz:.2f} herr={herr:.2f} stall={self.stall}")
            elif self.last_oz is not None and (
                    self.last_oz <= self.blind_entry or self.centered_descent):
                self.state = BLIND
                self.get_logger().info(
                    f"tag lost at oz={self.last_oz:.2f} "
                    f"centered_descent={self.centered_descent}: BLIND")
            else:
                self.state = HOLD
                self.get_logger().info("tag lost high: HOLD")

        elif self.state == BLIND:
            vz = self.sink
            self._stall(now)
            if self.stall >= 2:
                self._handoff(f"blind stall={self.stall}")
            elif fresh:
                self.state = TRACK
                self.get_logger().info("tag reacquired: TRACK")

        elif self.state == HOLD:
            if self.z_hold is None and self.local_z is not None:
                self.z_hold = self.local_z
            if self.z_hold is not None and self.local_z is not None:
                vz = max(-0.25, min(0.25, 0.6 * (self.z_hold - self.local_z)))
            if fresh:
                self.state = TRACK
                self.z_hold = None
                self.stale_since = None
                self.get_logger().info("tag reacquired: TRACK")
            else:
                if self.stale_since is None:
                    self.stale_since = now
                elif ((now - self.stale_since).nanoseconds * 1e-9 > 8.0
                        and self.gate is not None and self.reacq < 2):
                    self.reacq += 1
                    # Offset 2m toward pad to get it into camera FOV
                    pad_local = (-1.0, 0.0)
                    dx = pad_local[0] - self.gate[0]
                    dy = pad_local[1] - self.gate[1]
                    dist = (dx**2 + dy**2)**0.5
                    if dist > 0:
                        dx, dy = dx / dist * 2.0, dy / dist * 2.0
                    self.waypoints = [(self.gate[0] + dx, self.gate[1] + dy, self.gate[2])]
                    self.wp_idx = 0
                    self.state = WAYPOINT
                    self.oz_target = None
                    self.centered_descent = False
                    self.stale_since = None
                    self.get_logger().info(
                        f"reacquire {self.reacq}: returning to gate")

        if self.state == DONE:
            self.done_ticks += 1
            if self.done_ticks <= 10 and self.done_ticks % 2 == 0:
                cmd = VehicleCommand()
                cmd.timestamp = ts
                cmd.command = 21  # NAV_LAND
                cmd.target_system = 1
                cmd.target_component = 1
                cmd.from_external = True
                self.pub_cmd.publish(cmd)
            sec = now.nanoseconds // 1_000_000_000
            if sec != self.last_log:
                self.last_log = sec
                self.get_logger().info("state=DONE awaiting PX4 touchdown")
            return  # stop the offboard stream so PX4 owns the landing

        mode = OffboardControlMode()
        mode.timestamp = ts
        mode.position = False
        mode.velocity = True
        mode.acceleration = False
        self.pub_mode.publish(mode)
        sp = TrajectorySetpoint()
        sp.timestamp = ts
        sp.position = [float("nan")] * 3
        sp.velocity = [vx, vy, vz]
        sp.yaw = float("nan")
        self.pub_sp.publish(sp)

        sec = now.nanoseconds // 1_000_000_000
        if sec != self.last_log:
            self.last_log = sec
            o = f"({self.opt[0]:+.2f},{self.opt[1]:+.2f},{self.opt[2]:.2f})" if self.opt else "NONE"
            t = f"{self.oz_target:.2f}" if self.oz_target is not None else "--.--"
            self.get_logger().info(
                f"state={self.state} nav={self.nav} opt={o} tgt={t} yaw={self.heading:+.0f} "
                f"cmd=({vx:+.2f},{vy:+.2f},{vz:+.2f}) stall={self.stall}")


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
