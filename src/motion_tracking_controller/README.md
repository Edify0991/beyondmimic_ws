# BeyondMimic Motion Tracking Inference

[[Website]](https://beyondmimic.github.io/)
[[Arxiv]](https://arxiv.org/abs/2508.08241)
[[Video]](https://youtu.be/RS_MtKVIAzY)

This repository provides the inference pipeline for motion tracking policies in BeyondMimic. The pipeline is implemented
in C++ using the ONNX CPU inference engine. Model parameters (joint order, impedance, etc.)
are stored in ONNX metadata, and the reference motion is returned via the `forward()` function.
See [this script](https://github.com/HybridRobotics/whole_body_tracking/blob/main/source/whole_body_tracking/whole_body_tracking/utils/exporter.py)
for details on exporting models.

This repo also serves as an example of how to implement a custom controller using the
[legged_control2](https://qiayuanl.github.io/legged_control2_doc/) framework.

## Installation

`legged_control2` and Unitree prebuilt packages are currently published for **ROS 2 Jazzy / Ubuntu 24.04 (noble)**.
If you are on **ROS 2 Humble / Ubuntu 22.04 (jammy)**, the old `jammy-humble-*` buildfarm paths may return `404`.

### Option A (Recommended): Native Jazzy on Ubuntu 24.04

```bash
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions \
  python3-vcstool \
  python3-rosdep \
  python3-catkin-pkg \
  git

source /opt/ros/jazzy/setup.bash
echo "ROS_DISTRO=$ROS_DISTRO"   # should print jazzy
```

```bash
sudo rosdep init || true
rosdep update
```

```bash
# Choose buildfarm suffix by CPU architecture:
# amd64 -> noble-jazzy-amd64
# arm64 -> noble-jazzy-arm64

ARCH="$(dpkg --print-architecture)"
if [ "$ARCH" = "amd64" ]; then
  BF="noble-jazzy-amd64"
elif [ "$ARCH" = "arm64" ]; then
  BF="noble-jazzy-arm64"
else
  echo "Unsupported arch: $ARCH" && exit 1
fi

echo "deb [trusted=yes] https://github.com/qiayuanl/legged_buildfarm/raw/${BF}/ ./" | sudo tee /etc/apt/sources.list.d/qiayuanl_legged_buildfarm.list
echo "yaml https://github.com/qiayuanl/legged_buildfarm/raw/${BF}/local.yaml jazzy" | sudo tee /etc/ros/rosdep/sources.list.d/1-qiayuanl_legged_buildfarm.list

echo "deb [trusted=yes] https://github.com/qiayuanl/unitree_buildfarm/raw/${BF}/ ./" | sudo tee /etc/apt/sources.list.d/qiayuanl_unitree_buildfarm.list
echo "yaml https://github.com/qiayuanl/unitree_buildfarm/raw/${BF}/local.yaml jazzy" | sudo tee /etc/ros/rosdep/sources.list.d/1-qiayuanl_unitree_buildfarm.list

sudo apt-get update
sudo apt-get install -y \
  ros-jazzy-legged-control-base \
  ros-jazzy-unitree-description \
  ros-jazzy-unitree-systems
```

For simulation (MuJoCo), add simulation buildfarm (`amd64` only):

```bash
echo "deb [trusted=yes] https://github.com/qiayuanl/simulation_buildfarm/raw/noble-jazzy-amd64/ ./" | sudo tee /etc/apt/sources.list.d/qiayuanl_simulation_buildfarm.list
echo "yaml https://github.com/qiayuanl/simulation_buildfarm/raw/noble-jazzy-amd64/local.yaml jazzy" | sudo tee /etc/ros/rosdep/sources.list.d/1-qiayuanl_simulation_buildfarm.list
sudo apt-get update
sudo apt-get install -y ros-jazzy-mujoco-ros2-control
```

### Option B: Keep Humble Host, Build/Run in Jazzy Container

If your host must stay on Humble, use a Jazzy container for dependency installation and build.

```bash
docker run --rm -it --net=host --ipc=host \
  -v ~/colcon_ws:/workspace \
  osrf/ros:jazzy-desktop
```

Inside container, run the same Jazzy commands from Option A, then build in `/workspace`.

### Build This Package

```bash
mkdir -p ~/colcon_ws/src
cd ~/colcon_ws/src
git clone https://github.com/qiayuanl/unitree_bringup.git
git clone https://github.com/HybridRobotics/motion_tracking_controller.git
cd ~/colcon_ws
source /opt/ros/jazzy/setup.bash
```

```bash
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelwithDebInfo --packages-up-to unitree_bringup
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelwithDebInfo --packages-up-to motion_tracking_controller
source install/setup.bash
```

If Conda Python interferes (`ModuleNotFoundError: catkin_pkg`), switch to system Python:

```bash
unset CONDA_PREFIX CONDA_DEFAULT_ENV PYTHONPATH
source /opt/ros/jazzy/setup.bash
```

## Basic Usage

### Sim-to-Sim

We provide a launch file for running the policy in MuJoCo simulation.

```bash
# Recommended: load policy from local ONNX file (absolute path or `~`)
ros2 launch motion_tracking_controller mujoco.launch.py policy_path:=<your_onnx_file_path>
```

```bash
# Optional: use local reference motion file (in addition to local ONNX policy)
ros2 launch motion_tracking_controller mujoco.launch.py \
  policy_path:=<your_onnx_file_path> \
  reference_path:=<your_reference_csv_path> \
  reference_loop:=true
```

```bash
# Optional: load policy from WandB (explicitly enable it)
ros2 launch motion_tracking_controller mujoco.launch.py \
  use_wandb:=true \
  wandb_path:=<your_wandb_run_path>
```

### Real Experiments

> ⚠️ **Disclaimer**  
> Running these models on real robots is **dangerous** and entirely at your own risk.  
> They are provided **for research only**, and we accept **no responsibility** for any harm, damage, or malfunction.

1. Connect to the robot via ethernet cable.
2. Set the ethernet adapter to static IP: `192.168.123.11`.
3. Use `ifconfig` to find the `<network_interface>`, (e.g.,`eth0` or `enp3s0`).

```bash
# Recommended: load policy from local ONNX file (absolute path or `~`)
ros2 launch motion_tracking_controller real.launch.py \
  network_interface:=<network_interface> \
  policy_path:=<your_onnx_file_path>
```

```bash
# Optional: use local reference motion file (in addition to local ONNX policy)
ros2 launch motion_tracking_controller real.launch.py \
  network_interface:=<network_interface> \
  policy_path:=<your_onnx_file_path> \
  reference_path:=<your_reference_csv_path> \
  reference_loop:=false
```

```bash
# Optional: load policy from WandB (explicitly enable it)
ros2 launch motion_tracking_controller real.launch.py \
  network_interface:=<network_interface> \
  use_wandb:=true \
  wandb_path:=<your_wandb_run_path>
```

The robot should enter standby controller in the beginning.
Use the Unitree remote (joystick) to start and stop the policy:

- Standby controller (joint position control): `L1 + A`
- Motion tracking controller (the policy): `R1 + A`
- E-stop (damping): `B`

## Code Structure

This section will be especially helpful if you decide to write your own legged_control2 controller.
For a minimal starting point, check
the [legged_template_controller](https://github.com/qiayuanl/legged_template_controller).

Below is an overview of the code structure for this repository:

- **`include`** or **`src`**
    - **`MotionTrackingController`** Manages observations (like an RL environment) and passes them to the policy.

    - **`MotionOnnxPolicy`** Wraps the neural network, runs inference, and reads reference motion from ONNX or an optional local file.

    - **`MotionCommand`** Defines observation terms aligned with the training code.


- **`launch`**
    - Includes launch files like `mujoco.launch.py` and `real.launch.py` for simulation and real robot execution.
- **`config`**
    - Stores configuration files for standby controller and state estimation params.

## Local Reference Motion File Format

If `reference_path` is set, the controller still uses ONNX for action inference, but reference motion will be loaded from your local file.

- One frame per line
- Skip empty lines and lines starting with `#`
- Use 4 fields separated by `;`
- Field order:
  1. `joint_pos`: comma-separated list, length = number of robot joints
  2. `joint_vel`: comma-separated list, length = number of robot joints
  3. `body_pos`: comma-separated list, length = `3 * num_bodies` (xyz per body)
  4. `body_quat`: comma-separated list, length = `4 * num_bodies` (wxyz per body)

Example (for illustration only):

```text
# joint_pos ; joint_vel ; body_pos ; body_quat
0.1,0.2 ; 0.0,0.0 ; 0,0,1, 1,0,1 ; 1,0,0,0, 1,0,0,0
```
