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
