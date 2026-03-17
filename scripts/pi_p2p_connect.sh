#!/bin/bash
# Pi P2P WiFi Direct Connect
# Run: bash ~/pi_p2p_connect.sh
# Run within 60 seconds after TurtleBot p2p_setup.sh completes

# TurtleBot P2P BSSID (fixed)
BSSID="2e:cf:67:0c:79:f7"

set -e

echo "[1/6] Removing existing 10.0.0.2 from wlan0..."
sudo ip addr del 10.0.0.2/24 dev wlan0 2>/dev/null || true

echo "[2/6] Scanning for DIRECT network..."
sudo wpa_cli -i wlan0 scan
sleep 3

echo "[3/6] Checking for DIRECT network..."
if ! sudo wpa_cli -i wlan0 scan_results | grep -q DIRECT; then
  echo "ERROR: No DIRECT network found. Run TurtleBot p2p_setup.sh first."
  exit 1
fi

# Use hardcoded BSSID, fallback to scan if needed
BSSID_FOUND=$(sudo wpa_cli -i wlan0 scan_results | grep DIRECT | awk '{print $1}' | head -1)
if [ -n "$BSSID_FOUND" ]; then
  BSSID="$BSSID_FOUND"
fi
echo "    Connecting to BSSID: $BSSID"

echo "[4/6] WPS PBC connection..."
if ! sudo wpa_cli -i wlan0 wps_pbc "$BSSID" | grep -q OK; then
  echo "ERROR: wps_pbc failed"
  exit 1
fi

echo "[5/6] Waiting for connection (15 sec)..."
sleep 15

if ! sudo wpa_cli -i wlan0 status | grep -q "wpa_state=COMPLETED"; then
  echo "ERROR: Connection not completed. WPS may have timed out."
  exit 1
fi

echo "[6/6] Setting IP 10.0.0.2/24..."
sudo ip addr add 10.0.0.2/24 dev wlan0 2>/dev/null || true

echo "Verifying connectivity..."
if ! ping -c 3 -W 5 10.0.0.1 > /dev/null 2>&1; then
  echo "WARNING: ping failed. Check TurtleBot P2P setup."
  exit 1
fi

echo ""
echo "=== P2P connected. You can now run python3 server.py ==="
