#!/usr/bin/env bash
# Node 2 (TurtleBot, Ubuntu/netplan): join the laptop hotspot "adown-net" with a fixed extra address 192.168.137.220.
set -e
sudo tee /etc/netplan/60-adown-net.yaml >/dev/null <<'NP'
network:
  version: 2
  wifis:
    wlan0:
      dhcp4: true
      optional: true
      addresses: [192.168.137.220/24]
      access-points:
        "adown-net":
          password: "12345678"
NP
sudo chmod 600 /etc/netplan/60-adown-net.yaml
sudo netplan generate
sudo netplan apply
CFG=~/AdownDriver---VIP/config/local.yaml
sed -i -E 's/^(  node1_ip:).*/\1 192.168.137.210        # cockpit on the laptop hotspot adown-net/; s/^(  node2_ip:).*/\1 192.168.137.220        # TurtleBot on adown-net/; s/^(  dashboard_ip:).*/\1 192.168.137.220    # dashboard runs on the TurtleBot/' "$CFG"
grep -E "node1_ip|node2_ip|dashboard_ip" "$CFG"
sudo systemctl restart adown-node2 adown-dashboard
sleep 8
ip -4 addr show wlan0 | grep inet || true
iwgetid -r || echo "(wlan0 not associated yet - is the laptop hotspot adown-net on?)"
echo NODE2_WIFI_OK
