# BeyondMimic Minimal Startup Checklist

## Scope
- Workspace: `~/wmd/beyondmimic_ws`
- Package: `src/motion_tracking_controller`
- Runtime: ROS 2 Jazzy container

## 0) Enter Container
```bash
sudo docker start -ai beyondmimic_jazzy
# If container does not exist yet:
# sudo docker run -it --name beyondmimic_jazzy --net=host --ipc=host \
#   -v "$HOME/wmd/beyondmimic_ws:/beyondmimic_ws" \
#   -v "$HOME/wmd/beyondmimic:/beyondmimic" \
#   m.daocloud.io/docker.io/library/ros:jazzy bash
```

## 1) Source Environment
```bash
source /opt/ros/jazzy/setup.bash
cd /beyondmimic_ws
source install/setup.bash
```

## 2) Convert BeyondMimic .npz to reference.csv

Script path:
- `/beyondmimic_ws/src/motion_tracking_controller/scripts/npz_to_reference_csv.py`

Basic conversion:
```bash
python3 /beyondmimic_ws/src/motion_tracking_controller/scripts/npz_to_reference_csv.py \
  --input /beyondmimic/data/motions/jumps1_subject1_seg1-267.npz \
  --output /beyondmimic/data/motions/jumps1_subject1_seg1-267_reference.csv
```

Common options:
```bash
python3 /beyondmimic_ws/src/motion_tracking_controller/scripts/npz_to_reference_csv.py \
  --input /beyondmimic/data/motions/xxx.npz \
  --output /beyondmimic/data/motions/xxx_reference.csv \
  --start-frame 0 \
  --end-frame -1 \
  --stride 1 \
  --target-fps 50 \
  --quat-order wxyz
```

Notes:
- BeyondMimic `jumps1_subject1_seg1-267.npz` fields (`joint_pos`, `joint_vel`, `body_pos_w`, `body_quat_w`) are directly supported.
- Output CSV format matches `motion_tracking_controller` local reference parser.

## 3) Sim2Sim Minimal Start
```bash
ros2 launch motion_tracking_controller mujoco.launch.py \
  policy_path:=/beyondmimic/logs/rsl_rl/g1_flat/2026-03-15_17-39-42/exported/policy.onnx
```

### Sim2Sim (with local reference)
```bash
ros2 launch motion_tracking_controller mujoco.launch.py \
  policy_path:=/beyondmimic/logs/rsl_rl/g1_flat/2026-03-15_17-39-42/exported/policy.onnx \
  reference_path:=/beyondmimic/data/motions/jumps1_subject1_seg1-267_reference.csv \
  reference_loop:=true
```

## 4) Real Robot Minimal Start
```bash
# find interface first, e.g. eth0/enp3s0
ip -br addr

ros2 launch motion_tracking_controller real.launch.py \
  network_interface:=<your_network_interface> \
  policy_path:=/beyondmimic/logs/rsl_rl/g1_flat/2026-03-15_17-39-42/exported/policy.onnx
```

### Real (with local reference)
```bash
ros2 launch motion_tracking_controller real.launch.py \
  network_interface:=<your_network_interface> \
  policy_path:=/beyondmimic/logs/rsl_rl/g1_flat/2026-03-15_17-39-42/exported/policy.onnx \
  reference_path:=/beyondmimic/data/motions/jumps1_subject1_seg1-267_reference.csv \
  reference_loop:=false
```

## 5) Docker Visualization (Important)

Can you visualize directly in current container?
- If current container was started **without** `DISPLAY` + `/tmp/.X11-unix` mount, MuJoCo GUI (`GLFW`) usually fails.
- You cannot add new mounts/env to an already-running container.

Choices:
1. Headless now (no restart needed):
```bash
apt-get update && apt-get install -y xvfb
xvfb-run -s "-screen 0 1280x720x24" \
  ros2 launch motion_tracking_controller mujoco.launch.py \
  policy_path:=/beyondmimic/logs/rsl_rl/g1_flat/2026-03-15_17-39-42/exported/policy.onnx
```
2. GUI visualization: restart container with X11 settings:
```bash
xhost +local:root
sudo docker run -it --name beyondmimic_jazzy_gui --net=host --ipc=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  -v "$HOME/wmd/beyondmimic_ws:/beyondmimic_ws" \
  -v "$HOME/wmd/beyondmimic:/beyondmimic" \
  m.daocloud.io/docker.io/library/ros:jazzy bash
```

## 6) ARM (NVIDIA Orin) Differences
- Architecture check: `dpkg --print-architecture` should be `arm64`.
- Buildfarm branch must use `noble-jazzy-arm64`.
- MuJoCo simulation packages are often `amd64`-only; on Orin prioritize real-controller pipeline.
- Prepare a separate `arm64` container/image for Orin.

## 7) Quick Troubleshooting
```bash
# Verify package visibility
ros2 pkg list | grep -E "motion_tracking_controller|unitree_bringup|legged_rl_controllers"

# If install/setup.bash missing or stale, rebuild:
cd /beyondmimic_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=RelwithDebInfo \
  --packages-up-to motion_tracking_controller
source install/setup.bash
```
