#!/usr/bin/env bash
# Fully reset the PEAK PCAN-USB adapter and bring can1 back up at 1 Mbit.
# Use when the adapter has accumulated errors and motors stop responding.
# Run with sudo.
set -uo pipefail

echo "==> before:"
grep -A2 "n -type-" /proc/pcan 2>/dev/null | tail -2

echo "==> taking can1 down"
ip link set can1 down 2>/dev/null

echo "==> reloading pcan driver"
modprobe -r pcan 2>&1 || echo "   (module busy or built-in; continuing)"
sleep 2
modprobe pcan 2>&1
sleep 2

echo "==> interfaces now:"
ip -br link show | grep -i can

IFACE=$(ls -1 /sys/class/net | grep -E '^can[0-9]+$' | while read -r i; do
  if [ -e "/sys/class/net/$i/device/../idVendor" ] || \
     grep -q "$i" /proc/pcan 2>/dev/null; then echo "$i"; fi
done | tail -1)
IFACE=${IFACE:-can1}
echo "==> using $IFACE"

ip link set "$IFACE" down 2>/dev/null
ip link set "$IFACE" type can bitrate 1000000 restart-ms 100
ip link set "$IFACE" up

echo "==> after:"
ip -details link show "$IFACE" | grep -E "can state|bitrate"
grep -A2 "n -type-" /proc/pcan 2>/dev/null | tail -2
echo
echo "Now rescan:"
echo "  python3 -m motorbridge scan --vendor robstride --channel $IFACE --transport socketcan --start-id 1 --end-id 7"
