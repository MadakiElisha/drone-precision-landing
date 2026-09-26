# Drone Precision Landing

Drone precision landing project.

## Architecture
- **Firmware:** PX4 v1.15.0 (External dependency at `~/PX4-Autopilot`)
- **Simulator:** Gazebo Sim 8.11 (via ROS 2 Jazzy)
- **Middleware:** ROS 2 Jazzy
- **Perception:** AprilTag (Classical baseline) -> ML Keypoint Detector

## PX4 tree modifications (one-time, machine-local)
- Airframe 4100_gz_x500_pl registered in
  ~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt
  (single added line: 4100_gz_x500_pl). ID range 4100-4199 is reserved for
  this project. ROMFS only installs airframes listed in that file.
  Undo: sed -i '/4100_gz_x500_pl/d' <that CMakeLists.txt>
- setup_symlinks.sh publishes world + models as symlinks and the airframe
  as a real copy (ROMFS ignores symlinks).

## Phase 4: autonomous precision landing (classical baseline) — ACHIEVED
Pipeline: gz camera -> ros_gz_bridge (one-way) -> ArUco detector
(pl_perception) -> landing_controller (pl_control) -> PX4 offboard velocity.

Measured conventions (do not re-derive, re-validate if camera mount changes):
- optical->body: bx=-oy, by=-ox, bz=-oz
- setpoint velocity passed in NED/FRD: (vx, vy) = (k*bx, -k*by)
- vertical loop closed on tag depth oz (PX4 offboard altitude hold drifts in SITL)

Flight protocol: ./run_sim.sh -> commander takeoff ->
ros2 launch pl_bringup camera_bridge.launch.py -> commander mode offboard.

Known limits: terminal standoff ~0.4-0.5 m (marker leaves reliable detection
range); final touchdown via commander land; sim RTF ~20% on WSL2 software
GL, so wall-clock runs ~5x slower than sim time.
