# Network

All traffic stays on the **dedicated travel router** (no campus Wi-Fi, no internet needed at the arena).

| Host | IP (static DHCP lease) | Role |
|---|---|---|
| Router | 192.168.8.1 | 5 GHz SSID `adown-net`, WPA2, channel fixed (avoid auto-hopping during trials) |
| Node 1 | 192.168.8.10 | cockpit DMS |
| Node 2 | 192.168.8.20 | TurtleBot + dashboard |
| Laptop | 192.168.8.100 | browser → http://192.168.8.20:8000 |

If your router uses another subnet, change `network.*_ip` in `config/default.yaml`.
The dashboard learns node IPs from incoming packets, so only `node2_ip` (on Node 1) and
`node1_ip` / `dashboard_ip` (on Node 2) really matter.

## UDP ports

| Port | Direction | Content | Rate |
|---|---|---|---|
| 5005 | Node 1 → Node 2 | `driver` packet: EAR, MAR, TH_EAR, C_driver, yaw, seq, timestamps, ground-truth label | 50 Hz (heartbeat) |
| 5006 | Node 2 → Node 1 | `vehicle_state`: state, buzzer/LED pattern, speed, echo of seq for RTT | 20 Hz |
| 5010 | nodes → dashboard | telemetry JSON | 10 Hz |
| 5011 | nodes → dashboard | JPEG previews (≤ 60 kB) | 6 Hz |
| 5020 | dashboard → Node 2 | commands (start/stop/estop/reset/set_param/run_start/run_stop/sim) | on demand |
| 5021 | dashboard → Node 1 | commands (calibrate/protocol_start/ack/mute/sim_behaviour) | on demand |
| 8000/tcp | browser → dashboard | HTTP + WebSocket | — |

Packets are JSON (≈ 300 bytes) with DSCP EF marking. The 50 Hz heartbeat is independent of
the camera frame rate: if Node 1's camera stalls, packets keep flowing but carry
`frame_age_ms` and are marked face-less after 300 ms → the FSM sees low C_driver
(→ SPEED_REDUCTION) instead of a link loss.

**Heartbeat budget (metric M5):** timeout 150 ms (7 missed packets at 50 Hz) + ≤ 33 ms control
period + motor publish ≈ 185 ms < 200 ms. Measured RTT on an idle 5 GHz router is 3–10 ms.

## Firewall

`ufw` is off on both images. If you enable it: `sudo ufw allow 5005:5021/udp && sudo ufw allow 8000/tcp`.

## ROS 2

Only Node 2 runs ROS 2 (`turtlebot3_bringup`). Set `ROS_DOMAIN_ID=30` so other teams' robots
on the same network cannot receive our `/cmd_vel`. Node 1 does not use ROS at all
(plain UDP keeps Node 1 on the stock Raspberry Pi OS image).
