#!/usr/bin/env bash
# Node 2 (TurtleBot 3 Burger) - Ubuntu 22.04 server + ROS 2 Humble (ROBOTIS e-Manual image).
# Usage:  cd ~/AdownDriver---VIP && bash deploy/setup_node2.sh
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
sudo apt-get update
sudo apt-get install -y python3-venv python3-pip i2c-tools v4l-utils python3-lgpio || true
# Pi Camera V2 on Ubuntu 22.04: legacy camera stack -> /dev/video0 (bcm2835-v4l2)
CFG=/boot/firmware/config.txt
if [ -f "$CFG" ] && ! grep -q "^start_x=1" "$CFG"; then
  echo -e "\n# AdownDriver: Pi Camera V2 via V4L2\nstart_x=1\ngpu_mem=128\ndtparam=i2c_arm=on" | sudo tee -a "$CFG"
  echo ">>> reboot required for the camera"
fi
python3 -m venv --system-site-packages "$REPO/.venv"     # system site-packages => rclpy from /opt/ros
"$REPO/.venv/bin/pip" install --upgrade pip
"$REPO/.venv/bin/pip" install -r "$REPO/requirements-node2.txt"
sudo usermod -aG i2c,video,dialout "$USER" || true
for unit in adown-node2 adown-dashboard; do
  sed "s#__REPO__#$REPO#g; s#__USER__#$USER#g" "$REPO/deploy/systemd/$unit.service" | sudo tee /etc/systemd/system/$unit.service >/dev/null
done
sudo systemctl daemon-reload
sudo systemctl enable adown-node2.service adown-dashboard.service
echo "Done. Check:  source /opt/ros/humble/setup.bash && $REPO/.venv/bin/python -m adowndriver check --role node2"
echo "Dashboard:    http://$(hostname -I | awk '{print $1}'):8000"
