#!/usr/bin/env bash
# Node 1 (cockpit, Raspberry Pi OS / NetworkManager): prefer the laptop hotspot "adown-net", fixed extra address 192.168.137.210.
set -e
CFG=~/AdownDriver---VIP/config/local.yaml
sed -i -E 's/^(  node1_ip:).*/\1 192.168.137.210        # cockpit on the laptop hotspot adown-net/; s/^(  node2_ip:).*/\1 192.168.137.220        # TurtleBot on adown-net/; s/^(  dashboard_ip:).*/\1 192.168.137.220    # dashboard runs on the TurtleBot/' "$CFG"
grep -E "node1_ip|node2_ip|dashboard_ip" "$CFG"
sudo nmcli con delete adown-net >/dev/null 2>&1 || true
sudo nmcli con add type wifi con-name adown-net ifname wlan0 ssid adown-net \
  wifi-sec.key-mgmt wpa-psk wifi-sec.psk 12345678 \
  ipv4.method auto ipv4.addresses 192.168.137.210/24 connection.autoconnect-priority 10 >/dev/null
echo "Switching Wi-Fi to adown-net in 3 s - this SSH session will drop. Reconnect: ssh pi@192.168.137.210"
sudo nohup bash -c 'sleep 3; nmcli con up adown-net; systemctl restart adown-node1' >/dev/null 2>&1 &
echo NODE1_WIFI_OK
