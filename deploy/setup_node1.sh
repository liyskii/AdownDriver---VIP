#!/usr/bin/env bash
# Node 1 (cockpit DMS) - Raspberry Pi OS Bookworm 64-bit on the IoT-kit Pi 4B.
# Usage:  cd ~/AdownDriver---VIP && bash deploy/setup_node1.sh
set -euo pipefail
REPO="$(cd "$(dirname "$0")/.." && pwd)"
sudo apt-get update
sudo apt-get install -y python3-venv python3-dev python3-lgpio i2c-tools v4l-utils libgl1 libegl1 libgles2 alsa-utils
sudo raspi-config nonint do_i2c 0 || true          # OLED on I2C-1 (SDA=GPIO2, SCL=GPIO3)
python3 -m venv --system-site-packages "$REPO/.venv"
"$REPO/.venv/bin/pip" install --upgrade pip
"$REPO/.venv/bin/pip" install -r "$REPO/requirements-node1.txt"
"$REPO/.venv/bin/python" - <<'PY'
from adowndriver.config import load_config
from adowndriver.driver_monitor.backend import ensure_model
print("model:", ensure_model(load_config().driver))
PY
sudo usermod -aG gpio,i2c,video "$USER" || true
sed "s#__REPO__#$REPO#g; s#__USER__#$USER#g" "$REPO/deploy/systemd/adown-node1.service" | sudo tee /etc/systemd/system/adown-node1.service >/dev/null
sudo systemctl daemon-reload
sudo systemctl enable adown-node1.service
echo "Done. Check:  $REPO/.venv/bin/python -m adowndriver check --role node1"
echo "Start now:    sudo systemctl start adown-node1   (logs: journalctl -u adown-node1 -f)"
