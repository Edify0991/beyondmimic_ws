xhost +si:localuser:root
sudo docker run -it --name beyondmimic_jazzy_gui --net=host --ipc=host \
  -e DISPLAY=$DISPLAY \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  --device /dev/dri \
  -v "$HOME/wmd/beyondmimic_ws:/beyondmimic_ws" \
  -v "$HOME/wmd/beyondmimic:/beyondmimic" \
  beyondmimic:jazzy-amd64 bash

# 让 shell 脚本在任一命令返回非 0（失败）时立即退出
set -e

# 1) 用代理让 apt 访问 github（把 7890 改成你实际代理端口）
cat >/etc/apt/apt.conf.d/90proxy <<'EOF'
Acquire::http::Proxy "http://127.0.0.1:7890";
Acquire::https::Proxy "http://127.0.0.1:7890";
Acquire::ForceIPv4 "true";
EOF

# 2) 恢复官方 buildfarm 源（不要镜像 github-raw）
rm -f /etc/apt/sources.list.d/*qiayuanl*buildfarm*.list
cat >/etc/apt/sources.list.d/qiayuanl_legged_buildfarm.list <<'EOF'
deb [trusted=yes] https://github.com/qiayuanl/legged_buildfarm/raw/noble-jazzy-amd64/ ./
EOF
cat >/etc/apt/sources.list.d/qiayuanl_unitree_buildfarm.list <<'EOF'
deb [trusted=yes] https://github.com/qiayuanl/unitree_buildfarm/raw/noble-jazzy-amd64/ ./
EOF

# 3) 连通性自检（应返回 200/301/302）
curl -I --proxy http://127.0.0.1:7890 https://github.com
curl -I --proxy http://127.0.0.1:7890 https://github.com/qiayuanl/unitree_buildfarm/raw/noble-jazzy-amd64/Packages

# 4) 更新并验证包是否可见
apt-get clean
apt-get update
apt-cache search '^ros-jazzy-legged' | head -n 20
apt-cache search '^ros-jazzy-unitree' | head -n 20
```

echo "deb [trusted=yes] https://github.com/qiayuanl/legged_buildfarm/raw/noble-jazzy-amd64/ ./" | sudo tee /etc/apt/sources.list.d/qiayuanl_legged_buildfarm.list
echo "yaml https://github.com/qiayuanl/legged_buildfarm/raw/noble-jazzy-amd64/local.yaml jazzy" | sudo tee /etc/ros/rosdep/sources.list.d/1-qiayuanl_legged_buildfarm.list
sudo apt-get update

# Add apt source
echo "deb [trusted=yes] https://github.com/qiayuanl/simulation_buildfarm/raw/noble-jazzy-amd64/ ./" | sudo tee /etc/apt/sources.list.d/qiayuanl_simulation_buildfarm.list
echo "yaml https://github.com/qiayuanl/simulation_buildfarm/raw/noble-jazzy-amd64/local.yaml jazzy" | sudo tee /etc/ros/rosdep/sources.list.d/1-qiayuanl_simulation_buildfarm.list
sudo apt-get update

# Install MuJoCo
sudo apt install ros-jazzy-mujoco-ros2-control

echo "deb [trusted=yes] https://github.com/qiayuanl/unitree_buildfarm/raw/noble-jazzy-amd64/ ./" | sudo tee /etc/apt/sources.list.d/qiayuanl_unitree_buildfarm.list
echo "yaml https://github.com/qiayuanl/unitree_buildfarm/raw/noble-jazzy-amd64/local.yaml jazzy" | sudo tee /etc/ros/rosdep/sources.list.d/1-qiayuanl_unitree_buildfarm.list
sudo apt-get update


# Install packages
sudo apt-get install ros-jazzy-unitree-description
sudo apt-get install ros-jazzy-unitree-systems

rosdep install --from-paths src --ignore-src -r -y

colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelwithDebInfo --packages-up-to unitree_bringup
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=RelwithDebInfo --packages-up-to motion_tracking_controller
source install/setup.bash

# 2) 给 apt 配代理 + 强制 IPv4
cat >/etc/apt/apt.conf.d/90proxy <<'EOF'
Acquire::http::Proxy "http://127.0.0.1:7890";
Acquire::https::Proxy "http://127.0.0.1:7890";
Acquire::ForceIPv4 "true";
EOF

# 3) 仅保留官方 buildfarm 源
rm -f /etc/apt/sources.list.d/*qiayuanl*buildfarm*.list
cat >/etc/apt/sources.list.d/qiayuanl_legged_buildfarm.list <<'EOF'
deb [trusted=yes] https://github.com/qiayuanl/legged_buildfarm/raw/noble-jazzy-amd64/ ./
EOF
cat >/etc/apt/sources.list.d/qiayuanl_unitree_buildfarm.list <<'EOF'
deb [trusted=yes] https://github.com/qiayuanl/unitree_buildfarm/raw/noble-jazzy-amd64/ ./
EOF

apt-get clean
apt-get update
apt-get install -y ros-jazzy-legged-control-base ros-jazzy-unitree-description ros-jazzy-unitree-systems

sudo apt install ros-jazzy-mujoco-ros2-control

source /opt/ros/jazzy/setup.bash
ros2 pkg list | grep -E '^legged_rl_controllers$' || true
ls /opt/ros/jazzy/include/legged_rl_controllers || true

ros2 launch motion_tracking_controller mujoco.launch.py \
  policy_path:=/beyondmimic/logs/rsl_rl/g1_flat/2026-03-15_17-39-42/exported/policy.onnx \
  reference_path:=/beyondmimic/data/motions/your_reference.csv \
  reference_loop:=true