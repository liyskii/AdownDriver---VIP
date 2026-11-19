#!/usr/bin/env bash
# TurtleBot3 Burger base setup: Ubuntu Server 22.04 + ROS 2 Humble + TB3 packages (LDS-03) + OpenCR firmware.
# Usage on the TurtleBot Pi:  bash setup_tb3_base.sh 2>&1 | tee ~/tb3_setup.log
set -euo pipefail
LDS=${LDS_MODEL:-LDS-03}
step() { echo; echo "=================== $* ==================="; }

step "1/6 wait for automatic apt updates to finish"
while sudo fuser /var/lib/dpkg/lock-frontend >/dev/null 2>&1; do echo "apt is busy, waiting 10 s..."; sleep 10; done

step "2/6 system update"
sudo apt-get update
sudo DEBIAN_FRONTEND=noninteractive apt-get -y upgrade
sudo apt-get install -y avahi-daemon software-properties-common curl git
sudo systemctl mask sleep.target suspend.target hibernate.target hybrid-sleep.target

step "3/6 ROS 2 Humble (ros-base)"
if [ ! -f /opt/ros/humble/setup.bash ]; then
  sudo add-apt-repository -y universe
  V=$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F tag_name | awk -F\" '{print $4}')
  curl -L -o /tmp/ros2-apt-source.deb "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${V}/ros2-apt-source_${V}.$(. /etc/os-release && echo $VERSION_CODENAME)_all.deb"
  sudo dpkg -i /tmp/ros2-apt-source.deb
  sudo apt-get update
  sudo apt-get install -y ros-humble-ros-base ros-dev-tools
fi

step "4/6 TurtleBot3 packages"
sudo apt-get install -y python3-colcon-common-extensions python3-argcomplete libboost-system-dev build-essential \
  ros-humble-hls-lfcd-lds-driver ros-humble-turtlebot3-msgs ros-humble-dynamixel-sdk libudev-dev
mkdir -p ~/turtlebot3_ws/src && cd ~/turtlebot3_ws/src
[ -d turtlebot3 ] || git clone -b humble https://github.com/ROBOTIS-GIT/turtlebot3.git
[ -d coin_d4_driver ] || git clone -b humble https://github.com/ROBOTIS-GIT/coin_d4_driver.git
[ -d ld08_driver ] || git clone -b humble https://github.com/ROBOTIS-GIT/ld08_driver.git
rm -rf turtlebot3/turtlebot3_cartographer turtlebot3/turtlebot3_navigation2
cd ~/turtlebot3_ws
set +u; source /opt/ros/humble/setup.bash; set -u
sudo apt-get install -y ros-humble-xacro ros-humble-robot-state-publisher python3-rosdep
[ -f /etc/ros/rosdep/sources.list.d/20-default.list ] || sudo rosdep init
rosdep update
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --parallel-workers 2

step "5/6 environment + udev"
grep -q "turtlebot3_ws/install" ~/.bashrc || cat >> ~/.bashrc <<RC
source /opt/ros/humble/setup.bash
source ~/turtlebot3_ws/install/setup.bash
export ROS_DOMAIN_ID=30
export TURTLEBOT3_MODEL=burger
export LDS_MODEL=$LDS
RC
set +u; source ~/turtlebot3_ws/install/setup.bash; set -u
sudo cp "$(ros2 pkg prefix turtlebot3_bringup)/share/turtlebot3_bringup/script/99-turtlebot3-cdc.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
sudo usermod -aG dialout "$USER" || true

step "6/6 OpenCR firmware"
sudo dpkg --add-architecture armhf
sudo apt-get update && sudo apt-get install -y libc6:armhf
cd ~ && rm -rf opencr_update opencr_update.tar.bz2
wget -q https://github.com/ROBOTIS-GIT/OpenCR-Binaries/raw/master/turtlebot3/ROS2/latest/opencr_update.tar.bz2
tar -xf opencr_update.tar.bz2
if [ -e /dev/ttyACM0 ]; then
  cd ~/opencr_update && ./update.sh /dev/ttyACM0 burger.opencr
else
  echo "!!! OpenCR not found on /dev/ttyACM0 - connect it by USB, power it on, then run:"
  echo "    cd ~/opencr_update && ./update.sh /dev/ttyACM0 burger.opencr"
fi

echo; echo "DONE. Reboot (sudo reboot), then test:"
echo "  ros2 launch turtlebot3_bringup robot.launch.py      # window 1"
echo "  ros2 topic echo /scan --once                        # window 2"
echo "  ros2 run turtlebot3_teleop teleop_keyboard          # window 2"
