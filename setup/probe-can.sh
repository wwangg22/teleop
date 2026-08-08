#!/usr/bin/env bash
# Determine the CAN bitrate the reBot B601-RS arm is running at.
# Tries 1 Mbit first (RobStride default), falls back to 500k / 250k.
# Harmless: a wrong bitrate just produces error frames, it cannot damage hardware.
set -uo pipefail

IFACE=can1   # PEAK PCAN-USB. can0 is the Orin's onboard mttcan (unused).

probe() {
  local rate=$1
  echo
  echo "=================================================="
  echo "  Probing ${IFACE} @ ${rate} bit/s"
  echo "=================================================="

  ip link set "$IFACE" down 2>/dev/null
  if ! ip link set "$IFACE" type can bitrate "$rate"; then
    echo "  !! could not set bitrate ${rate}"; return 1
  fi
  ip link set "$IFACE" up || { echo "  !! could not bring up ${IFACE}"; return 1; }

  # Baseline error counters
  local before
  before=$(ip -statistics link show "$IFACE" | awk '/RX:/{getline; print $3}')

  echo "  Listening 5s -- move the arm by hand if it is idle..."
  timeout 5 candump -n 20 "$IFACE" || true

  local after state
  after=$(ip -statistics link show "$IFACE" | awk '/RX:/{getline; print $3}')
  state=$(ip -details link show "$IFACE" | grep -oE 'state [A-Z-]+' | head -1)

  echo
  echo "  bus state : ${state:-unknown}"
  echo "  rx errors : ${before:-0} -> ${after:-0}"
  ip link set "$IFACE" down 2>/dev/null
}

echo "Make sure the arm is POWERED ON before running this."
echo

for rate in 1000000 500000 250000; do
  probe "$rate"
done

echo
echo "=================================================="
echo "HOW TO READ THIS:"
echo "  Correct bitrate  -> real frames printed, state ERROR-ACTIVE,"
echo "                      rx error count stays flat."
echo "  Wrong bitrate    -> no frames or garbage, state ERROR-PASSIVE"
echo "                      or BUS-OFF, rx error count climbs fast."
echo "=================================================="
