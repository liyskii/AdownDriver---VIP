#!/bin/bash
# Health check of the cockpit (Node 1). Run by demo.ps1 -Check.
sec() { echo; echo "=================== $1"; }
sec "cockpit service"; systemctl is-active adown-node1
sec "cameras";        ls -l /dev/video* 2>&1 | head
sec "audio";          aplay -l 2>&1 | head -8
sec "power";          vcgencmd get_throttled 2>/dev/null; vcgencmd measure_temp 2>/dev/null
sec "node1 log";      journalctl -u adown-node1 -n 30 --no-pager 2>&1 | tail -30
