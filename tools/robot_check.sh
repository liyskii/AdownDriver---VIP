#!/bin/bash
# Full health check of the robot (Node 2). Run by demo.ps1 -Check; output goes to check_report.txt on the laptop.
source /opt/ros/humble/setup.bash 2>/dev/null
source ~/turtlebot3_ws/install/setup.bash 2>/dev/null
export ROS_DOMAIN_ID=30 TURTLEBOT3_MODEL=burger LDS_MODEL=LDS-03
R=~/AdownDriver---VIP
sec() { echo; echo "=================== $1"; }
sec "time / uptime";  date; uptime
sec "services";       for s in turtlebot3-bringup adown-node2 adown-dashboard; do echo "$s: $(systemctl is-active $s)"; done
sec "code version";   cat $R/.synced 2>/dev/null; grep -c RESUBSCRIBE_S $R/adowndriver/vehicle/io.py
sec "local.yaml";     cat $R/config/local.yaml
sec "usb devices";    ls -l /dev/ttyUSB* /dev/ttyACM* /dev/tb3_* /dev/video* 2>&1
sec "power";          vcgencmd get_throttled 2>/dev/null; vcgencmd measure_temp 2>/dev/null
sec "battery";        timeout 6 ros2 topic echo --once /battery_state 2>&1 | grep -E "voltage|percentage" || echo "no /battery_state"
sec "ros topics";     timeout 8 ros2 topic list 2>&1
sec "scan publishers/subscribers"; timeout 8 ros2 topic info -v /scan 2>&1 | grep -E "Node name|Reliability|Durability|count|Topic type"
sec "scan rate (8 s)"; timeout 9 ros2 topic hz /scan 2>&1 | tail -4
sec "one scan";       timeout 6 ros2 topic echo --once /scan --field range_max 2>&1 | head -3
sec "bringup log (lidar)"; journalctl -u turtlebot3-bringup -n 300 --no-pager 2>&1 | grep -iE "lidar|coin|error|warn|died|exit|data check" | tail -25
sec "node2 log";      journalctl -u adown-node2 -n 40 --no-pager 2>&1 | tail -40
sec "dashboard log";  journalctl -u adown-dashboard -n 10 --no-pager 2>&1 | tail -10
sec "dmesg usb";      dmesg 2>/dev/null | grep -iE "ttyUSB|ttyACM|usb .*disconnect|under-voltage" | tail -15
