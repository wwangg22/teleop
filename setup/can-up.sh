#!/usr/bin/env bash
# Bring up the reBot B601-RS CAN bus on the PEAK PCAN-USB adapter.
# can1 = PCAN-USB. can0 = Orin onboard mttcan (no transceiver, unused).
# Bitrate 1 Mbit/s, confirmed against the arm.
# Uses the /etc/sudoers.d/99-can-willy rule, so no password prompt.
set -euo pipefail

IFACE=can1
BITRATE=1000000

sudo ip link set "$IFACE" down 2>/dev/null || true
sudo ip link set "$IFACE" type can bitrate "$BITRATE"
sudo ip link set "$IFACE" up

ip -details link show "$IFACE" | sed -n '1,3p'
echo "$IFACE up @ ${BITRATE} bit/s"
