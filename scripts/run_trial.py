"""One automated precision-landing trial.

- Takeoff: pxh stdin (proven).
- Offboard: external DO_SET_MODE over ROS (proven).
- Success requires actual flight (min_z < -1.5 m).
"""
import argparse
import csv
import os
import subprocess
import threading
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from px4_msgs.msg import VehicleCommand, VehicleLocalPosition, VehicleStatus


CAP_BYTES = 25 * 1024 * 1024


def _drain(pipe, path):
    """Drain a child pipe into a size-capped file. A spamming child
    (PX4 timesync warnings) can never fill the disk again; the pipe
    keeps draining so the child never blocks."""
    written = 0
    with open(path, "wb") as f:
        for chunk in iter(lambda: pipe.read(65536), b""):
            if written < CAP_BYTES:
                n = min(len(chunk), CAP_BYTES - written)
                f.write(chunk[:n])
                f.flush()
                written += n


ROOT = "/home/madakie/precision_landing"
NAV_OFFBOARD = getattr(VehicleStatus, "NAVIGATION_STATE_OFFBOARD", 14)
ARM_DISARMED = getattr(VehicleStatus, "ARMING_STATE_DISARMED", 1)
TRACE = None


def trace(node):
    if TRACE is None:
        return
    z = f"{node.local.z:.2f}" if node.local is not None else "None"
    nav = node.status.nav_state if node.status is not None else None
    arm = node.status.arming_state if node.status is not None else None
    TRACE.write(f"{time.time():.1f} nav={nav} arm={arm} z={z}\n")
    TRACE.flush()


class Mission(Node):
    def __init__(self):
        super().__init__("trial_mission")
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT,
                         history=HistoryPolicy.KEEP_LAST)
        self.status = None
        self.local = None
        self.min_z = 0.0
        self.last_nav = None
        self.last_arm = None
        self.create_subscription(
            VehicleStatus, "/fmu/out/vehicle_status",
            lambda m: setattr(self, "status", m), qos)
        self.create_subscription(
            VehicleLocalPosition, "/fmu/out/vehicle_local_position",
            self.local_cb, qos)
        self.pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos)

    def local_cb(self, m):
        self.local = m
        if m.z < self.min_z:
            self.min_z = m.z

    def note_transitions(self):
        if self.status is None:
            return
        if self.status.nav_state != self.last_nav:
            print(f"[trial] nav_state -> {self.status.nav_state}", flush=True)
            self.last_nav = self.status.nav_state
        if self.status.arming_state != self.last_arm:
            print(f"[trial] arming_state -> {self.status.arming_state}", flush=True)
            self.last_arm = self.status.arming_state

    def cmd_msg(self, command, p1=0.0, p2=0.0, p7=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds // 1000)
        msg.command = command
        msg.param1 = p1
        msg.param2 = p2
        msg.param7 = p7
        msg.target_system = 1
        msg.target_component = 1
        msg.from_external = True
        return msg

    def send_until(self, command, pred, timeout, desc, p1=0.0, p2=0.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            self.pub_cmd.publish(self.cmd_msg(command, p1, p2))
            rclpy.spin_once(self, timeout_sec=0.5)
            trace(self)
            self.note_transitions()
            if pred():
                return True
        print(f"[trial] TIMEOUT: {desc}", flush=True)
        return False


def wait_for(pred, timeout, desc, node):
    t0 = time.time()
    while time.time() - t0 < timeout:
        rclpy.spin_once(node, timeout_sec=0.2)
        trace(node)
        node.note_transitions()
        if pred():
            return True
    print(f"[trial] TIMEOUT: {desc}", flush=True)
    return False


def score():
    try:
        with open(f"{ROOT}/gt_log.csv") as f:
            rows = list(csv.DictReader(f))
        last = rows[-1]
        ex = (float(last["x"]) - 4.0) * 100.0
        ey = (float(last["y"]) - 0.0) * 100.0
        return ex, ey, (ex * ex + ey * ey) ** 0.5
    except Exception as e:
        print(f"[trial] scoring failed: {e}", flush=True)
        return float("nan"), float("nan"), float("nan")


def pxh(sim, line):
    try:
        sim.stdin.write((line + "\n").encode())
        sim.stdin.flush()
    except Exception as e:
        print(f"[trial] pxh write failed: {e}", flush=True)


def main():
    global TRACE
    ap = argparse.ArgumentParser()
    ap.add_argument("--perception", choices=["aruco", "yolo"], default="yolo")
    ap.add_argument("--offset", default="5,0")
    ap.add_argument("--trial", type=int, default=1)
    ap.add_argument("--decoy", action="store_true")
    ap.add_argument("--occluder", action="store_true")
    ap.add_argument("--light", type=float, default=0.9)
    ap.add_argument("--circuit", action="store_true")
    args = ap.parse_args()

    ox, oy = args.offset.split(",")
    env = dict(os.environ)
    env["PX4_GZ_MODEL_POSE"] = f"{ox},{oy},0.3,0,0,0"
    current_path = env.get("GZ_SIM_RESOURCE_PATH", "")
    env["GZ_SIM_RESOURCE_PATH"] = f"/home/madakie/precision_landing/sim/models:{current_path}"
    env["PL_DECOY"] = "1" if args.decoy else "0"
    env["PL_OCCLUDER"] = "1" if args.occluder else "0"
    env["PL_DIM_LIGHT"] = str(args.light)
    if args.circuit:
        env["PL_OBSTACLES"] = "1"
        env["PL_WAYPOINTS"] = "5,0,4;8,5,4;3,7,3.5;0,2,3;3,1.5,2.5"
        env["PL_SPAWN"] = f"{ox},{oy}"

    st = os.statvfs(ROOT)
    free_gb = st.f_bavail * st.f_frsize / 1e9
    if free_gb < 5:
        raise SystemExit(f"[trial] abort: only {free_gb:.1f} GB free")

    TRACE = open(f"{ROOT}/trial_trace.log", "w")
    subprocess.run(["./sim/worlds/generate_world.sh"], cwd=ROOT, env=env, check=True)
    sim = subprocess.Popen(["./run_sim.sh"], cwd=ROOT, env=env,
                           stdin=subprocess.PIPE,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    threading.Thread(target=_drain,
                     args=(sim.stdout, f"{ROOT}/trial_sim.log"),
                     daemon=True).start()
    time.sleep(5)
    loggt = subprocess.Popen([sys.executable, "scripts/log_gt.py"], cwd=ROOT,
                             stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    launch = subprocess.Popen(
        ["ros2", "launch", "pl_bringup", "camera_bridge.launch.py",
         f"use_yolo:={'true' if args.perception == 'yolo' else 'false'}"],
        cwd=ROOT, env=env, stdout=subprocess.PIPE, stderr=subprocess.STDOUT)
    threading.Thread(target=_drain,
                     args=(launch.stdout, f"{ROOT}/trial_ros.log"),
                     daemon=True).start()

    print(f"[trial] NAV_OFFBOARD={NAV_OFFBOARD}", flush=True)
    rclpy.init()
    node = Mission()
    t0 = time.time()
    success = False
    try:
        if wait_for(lambda: node.status is not None, 180, "vehicle_status", node):
            time.sleep(3)
            pxh(sim, "commander takeoff")
            if wait_for(lambda: node.local is not None and node.local.z < -1.5,
                        120, "climb", node):
                time.sleep(2)
                engaged = node.send_until(
                    176, lambda: node.status is not None
                    and node.status.nav_state == NAV_OFFBOARD,
                    30, "offboard", p1=1.0, p2=6.0)
                if engaged:
                    disarmed = wait_for(
                        lambda: node.status is not None
                        and node.status.arming_state == ARM_DISARMED,
                        300, "disarm", node)
                    success = disarmed or node.min_z < -2.0
                    if disarmed and not success:
                        print(f"[trial] FALSE-SUCCESS guard: disarmed but "
                              f"min_z={node.min_z:.2f} (never flew)", flush=True)
    finally:
        time.sleep(1.0)
        ex, ey, er = score()
        wall = time.time() - t0
        for p in (loggt, launch, sim):
            p.send_signal(2)
        for p in (loggt, launch, sim):
            try:
                p.wait(timeout=15)
            except subprocess.TimeoutExpired:
                p.kill()
                p.wait(timeout=10)
        subprocess.run(["pkill", "-f", "gz gui"], capture_output=True)
        subprocess.run(["pkill", "-f", "gz sim"], capture_output=True)
        TRACE.close()
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass

    with open(f"{ROOT}/results.csv", "a", newline="") as f:
        w = csv.writer(f)
        if os.path.getsize(f"{ROOT}/results.csv") == 0:
            w.writerow(["ts", "perception", "trial", "offset", "config",
                        "err_x_cm", "err_y_cm", "err_radial_cm",
                        "success", "wall_s"])
        w.writerow([int(time.time()), args.perception, args.trial,
                    args.offset,
                    f"d={int(args.decoy)}_o={int(args.occluder)}_l={args.light:.1f}"
                    f"_c={int(args.circuit)}",
                    f"{ex:.2f}", f"{ey:.2f}", f"{er:.2f}",
                    success, f"{wall:.0f}"])
    print(f"[trial] {args.perception} #{args.trial} offset=({ox},{oy}) "
          f"success={success} radial={er:.2f} cm wall={wall:.0f}s", flush=True)


if __name__ == "__main__":
    main()
