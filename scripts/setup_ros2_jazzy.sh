#!/usr/bin/env bash
# Install ROS 2 Jazzy (minimal: ros-base + RViz2) on Ubuntu 24.04 (WSL2).
# Run once inside Ubuntu:  bash scripts/setup_ros2_jazzy.sh   (asks for your sudo password)
set -euo pipefail

echo "== 1/5 basic tools and locale =="
sudo apt-get update
sudo apt-get install -y software-properties-common curl locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "== 2/5 enable ROS 2 apt repository =="
sudo add-apt-repository -y universe
ROS_APT_SOURCE_VERSION=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F'"' '{print $4}')
CODENAME=$(. /etc/os-release && echo "$VERSION_CODENAME")
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
sudo dpkg -i /tmp/ros2-apt-source.deb
sudo apt-get update

echo "== 3/5 install ROS 2 Jazzy (ros-base + rviz2) and Python packages =="
sudo apt-get install -y ros-jazzy-ros-base ros-jazzy-rviz2 ros-dev-tools \
  python3-numpy python3-opencv python3-yaml python3-pytest

echo "== 4/5 load ROS 2 in every new terminal =="
grep -q 'source /opt/ros/jazzy/setup.bash' ~/.bashrc || echo 'source /opt/ros/jazzy/setup.bash' >> ~/.bashrc

echo "== 5/5 check =="
set +u  # ROS setup files read unset variables
source /opt/ros/jazzy/setup.bash
echo "ROS_DISTRO=${ROS_DISTRO}"
ros2 --help > /dev/null && echo "ros2 command OK"
df -h / | tail -1
echo "Done. Open a new terminal (or run: source ~/.bashrc)."
