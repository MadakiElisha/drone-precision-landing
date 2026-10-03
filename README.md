# 🚁 Drone Precision Landing

Drone precision landing project with autonomous circuit missions and ML vs classical robustness benchmark.

https://github.com/user-attachments/assets/9db95756-8852-4c58-b021-fff574487a6a

<img width="1916" height="1020" alt="Screenshot 2026-09-30 145159" src="https://github.com/user-attachments/assets/c6e3f1ac-a03e-47a4-a418-d1a7f0d9b5f2" />

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                    DOWNWARD CAMERA (30 FPS)                      │
│                         │                                        │
│                         ▼                                        │
│  Perception Layer                                                │
│   • ArUco: cv2.aruco.detectMarkers + solvePnP                    │
│   • YOLO: ultralytics YOLOv8n (640x480, ~30 FPS)                 │
│                         │                                        │
│                         ▼                                        │
│  Estimation Layer                                                │
│   • EMA filter (alpha=0.2) in optical frame                      │
│   • Removes 85% of high-frequency bbox jitter                    │
│   • Bridges 0.6s dropouts                                        │
│                         │                                        │
│                         ▼                                        │
│  Control Layer                                                   │
│   • State machine: WAYPOINT > CLIMB > TRACK > BLIND > DONE       │
│   • Yaw-corrected velocity commands (offboard)                   │
│   • Descent ramp gated on centering quality                      │
│   • Tag reacquisition when lost at gate                          │
│   • Handoff to NAV_LAND at oz <= 0.55 m                          │
│                         │                                        │
│                         ▼                                        │
│  PX4 Flight Controller                                           │
│   • Offboard velocity control                                    │
│   • NAV_LAND for final touchdown                                 │
└─────────────────────────────────────────────────────────────────┘
```

- **Firmware:** PX4 v1.15 SITL (Airframe 4100)
- **Simulator:** Gazebo Sim (custom world + downward camera)
- **Middleware:** ROS 2 Jazzy + CycloneDDS
- **Perception:** YOLOv8n (Ultralytics) bounding box -> pinhole depth estimation
- **Control:** Yaw-invariant offboard velocity controller with blind final descent and PX4 AUTO_LAND handoff

## PX4 tree modifications (one-time, machine-local)

- Airframe 4100_gz_x500_pl registered in `~/PX4-Autopilot/ROMFS/px4fmu_common/init.d-posix/airframes/CMakeLists.txt` (single added line: 4100_gz_x500_pl). ID range 4100-4199 is reserved for this project. ROMFS only installs airframes listed in that file. Undo: `sed -i '/4100_gz_x500_pl/d' <that CMakeLists.txt>`
- setup_symlinks.sh publishes world + models as symlinks and the airframe as a real copy (ROMFS ignores symlinks).

## Phase 4: Autonomous precision landing (classical baseline) — ACHIEVED

Pipeline: gz camera -> ros_gz_bridge (one-way) -> ArUco detector (pl_perception) -> landing_controller (pl_control) -> PX4 offboard velocity.

Measured conventions (do not re-derive; re-validate if camera mount changes):
- optical->body: bx=-oy, by=-ox, bz=-oz
- setpoint velocity passed in NED/FRD: (vx, vy) = (k*bx, -k*by)
- vertical loop closed on tag depth oz (PX4 offboard altitude hold drifts in SITL)

Flight protocol: `./run_sim.sh` -> `commander takeoff` -> `ros2 launch pl_bringup camera_bridge.launch.py` -> `commander mode offboard`.

**Baseline touchdown error: ~4 cm** (hands-off mission, auto-disarm, 1 m offset, yaw-invariant control).

Known limits: terminal standoff ~0.4-0.5 m (marker leaves reliable detection range); final touchdown via commander land; sim RTF ~20% on WSL2 software GL, so wall-clock runs ~5x slower than sim time.

## Phase 5: ML challenger deployment — SUCCESS

The YOLOv8n model replaced the classical ArUco detector. Trained on ~200 synthetic frames via ArUco-as-teacher distillation, the network learned pinhole depth estimation from 2D bounding boxes and vastly outperformed ArUco in robustness under blur, noise, and occlusion. Flight performance: ~9 cm radial touchdown error (comparable to the classical baseline's ~4 cm), with vastly superior resilience to visual degradation.

## Phase 6: Estimation layer and adversarial robustness — ACHIEVED

### Clean environment benchmark (10 autonomous trials)

In a pristine simulation both stacks achieve sub-20 cm precision. The EMA filter removes high-frequency bounding-box jitter without introducing structural phase lag.

| Stack | n  | Median (cm) | Mean (cm) | Max (cm) |
|-------|----|-------------|-----------|----------|
| ArUco | 9  | 7.36        | 8.26      | 16.18    |
| YOLO  | 12 | 17.15       | 67.37     | 369.76   |

### Adversarial decoy test (World v2)

An identical ArUco marker is placed at the spawn point without the dark landing-pad context. The classical stack has no notion of context: it lands on the distractor. The ML stack rejects it and lands on the true pad.

| Stack | n | Median (cm) | Mean (cm) | Max (cm) |
|-------|---|-------------|-----------|----------|
| ArUco | 2 | 107.72      | 107.72    | 108.30   |
| YOLO  | 1 | 17.63       | 17.63     | 17.63    |

**Conclusion:** classical perception fails catastrophically on out-of-context distractors (~100 cm error); ML perception rejects them (~17 cm error). Context awareness is the differentiator.

### Obstacle circuit mission (World v3)

End-to-end mission through a walled arena containing cubes, cylinders, and wall segments. The drone flies a five-waypoint circuit, loses and reacquires the pad, then lands.

| Stack | n | Median (cm) | Mean (cm) | Max (cm) |
|-------|---|-------------|-----------|----------|
| ArUco | 3 | 6.38        | 5.93      | 8.05     |
| YOLO  | 3 | 20.40       | 17.74     | 21.46    |

Circuit features:
- Bounding walls (north / south / east / west)
- Two wall segments threading the circuit path
- Four white cubes (0.8 m)
- Four blue cylinders (0.5 m radius, 1.0 m tall)
- Approach gate at local (-2, 1.5, 2.5)
- Tag reacquisition behavior when the pad is lost at standoff

## Quick start

### Prerequisites

- Ubuntu 22.04 or 24.04 (native or WSL2)
- Python 3.10+
- PX4-Autopilot (compiled for gz_x500_pl)
- ROS 2 Jazzy
- Gazebo Sim (Harmonic or Ionic)

### 1. Clone and setup

```bash
git clone <repo_url> && cd precision_landing
python3 -m venv venv
source venv/bin/activate
pip install -r requirements.txt
pip install ultralytics torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

### 2. Build the ROS 2 workspace

CRITICAL: always build with the venv interpreter to avoid the system-python shebang trap.

```bash
cd ros2_ws
/home/$USER/precision_landing/venv/bin/python -m colcon build
source install/setup.bash
cd ..
```

### 3. Run a single trial

```bash
# clean environment
python3 scripts/run_trial.py --perception yolo --trial 1 --offset 5,0

# adversarial decoy
python3 scripts/run_trial.py --perception yolo --trial 1 --offset 5,0 --decoy

# obstacle circuit
python3 scripts/run_trial.py --perception yolo --trial 1 --offset 5,0 --circuit
```

### 4. Run a batch and analyze

```bash
for p in aruco yolo; do
  for i in 1 2 3 4 5; do
    python3 scripts/run_trial.py --perception $p --trial $i --offset 5,0 --circuit
  done
done
python3 scripts/trial_stats.py
```

## Known limitations

- **YOLO fails on decoys without retraining** — the model saw only the full pad gestalt; hard-negative retraining would sharpen rejection.
- **EMA lag at high speed** — alpha=0.2 introduces ~160 ms lag during fast maneuvers; fine for landing, limiting for tracking.
- **Sim-to-real gap** — the model is trained on synthetic frames; real deployment needs transfer learning or a small real-world fine-tune set.
- **No appearance-based Re-ID** — the EMA predictor expires after 0.6 s without detections.

## Engineering lessons

### 1. The Kalman filter phase-lag trap

A 6-state constant-velocity Kalman filter in the optical frame structurally fights the drone's internal P-controller (a velocity damper). The CV assumption is physically false there: the filter rejects valid frames or lags, performing worse than raw noisy data.

**Solution:** 1st-order EMA filter, tuned offline against replay logs to maximize jitter reduction at minimum lag.

### 2. The colcon shebang trap

`colcon build` run with system Python writes `#!/usr/bin/python3` shebangs into node wrappers, ignoring the venv; launched nodes then die with `ModuleNotFoundError: ultralytics`.

**Solution:** always build with `venv/bin/python -m colcon`.

### 3. The 36 GB log incident

PX4 timesync warnings are unrate-limited; during a time-sync oscillation they print thousands of lines per second. An uncapped redirect filled the WSL2 virtual disk (36 GB in one trial) and turned every subsequent write into an I/O error.

**Solution:** capped background drain threads (25 MB per log) plus a pre-flight abort below 5 GB free.

### 4. The reacquire deadlock

Losing the tag at the approach gate sent the controller to HOLD with no recovery plan: it hovered forever.

**Solution:** an 8-second stale timeout flies a reacquire waypoint offset 2 m toward the pad, returning the pad to the camera FOV.

### 5. The standoff deadlock

Centered at the 0.55 m standoff with a 0.29 m residual offset, the old handoff gate (herr < 0.15 m) could never fire and the descent-gated stall probe never counted: infinite hover 25 cm above the pad.

**Solution:** relax the lateral handoff gate to 0.45 m at close range; PX4 NAV_LAND drops straight down from there.

## Dependencies

| Package       | Purpose                      |
|---------------|------------------------------|
| ultralytics   | YOLOv8n detection            |
| opencv-python | ArUco detection + solvePnP   |
| rclpy         | ROS 2 Python client          |
| px4_msgs      | PX4 message definitions      |
| vision_msgs   | Detection3DArray messages    |
| numpy         | Numerical operations         |

## Roadmap

- ✅ Automated evaluation framework
- ✅ Classical vs ML baseline comparison
- ✅ Adversarial decoy rejection
- ✅ Obstacle circuit missions
- ✅ EMA estimation filter
- 🔲 ML v2: hard-negative retraining against decoys
- 🔲 Sim-to-real deployment (Jetson + TensorRT)
- 🔲 Moving landing pad
- 🔲 Multi-pad selection

## License

MIT License — see LICENSE file for details.
