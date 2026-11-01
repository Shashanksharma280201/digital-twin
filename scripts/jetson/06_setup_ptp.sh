#!/usr/bin/env bash
# Makes the Orin the PTP grandmaster for the LiDARs (UDPv4, E2E, domain 0)
# and keeps the NIC clock and the system clock aligned.
# Usage: 06_setup_ptp.sh [iface] [--software]
#   --software  software timestamps even if the NIC offers hardware ones (NICs with a broken PHC).
# For PTP use the Orin's Intel ports (eth3-eth5, igc driver) in hardware mode. The built-in eth0
# (nvethernet) loses send timestamps: ~4 faults/min in hardware mode, ~1/s in software mode
# (tested 2026-10-22), so it is a fallback only; a 500 ms tx_timestamp_timeout cuts that to ~0.6/min.
set -euo pipefail
# shellcheck source=common.sh
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

step "06 PTP grandmaster"
force_sw=0; iface=""
for a in "$@"; do
  case "$a" in
    --software) force_sw=1 ;;
    *) iface="$a" ;;
  esac
done
[ -n "$iface" ] || iface="$(detect_wired_iface || true)"
[ -n "$iface" ] || die "Could not detect a wired interface; pass it explicitly: $0 eth0"
apt_install linuxptp ethtool

ts_mode="hardware"
if [ "$force_sw" -eq 1 ]; then
  ts_mode="software"
elif ! ethtool -T "$iface" 2>/dev/null | grep -q "hardware-transmit"; then
  warn "${iface}: no hardware-transmit timestamping reported -> using software timestamping (lower accuracy)"
  ts_mode="software"
fi
log "interface ${iface}, time_stamping ${ts_mode}"

conf=/etc/linuxptp/dt-gm.conf
sudo mkdir -p /etc/linuxptp
sudo tee "$conf" >/dev/null <<EOF
# Digital-twin rig: Orin is grandmaster for Mid-360 / Airy
[global]
gmCapable               1
priority1               10
domainNumber            0
network_transport       UDPv4
delay_mechanism         E2E
time_stamping           ${ts_mode}
tx_timestamp_timeout    500
logSyncInterval         0
[${iface}]
EOF
ok "wrote ${conf}"

ptp4l_bin="$(command -v ptp4l)"
phc2sys_bin="$(command -v phc2sys)"

sudo tee /etc/systemd/system/dt-ptp4l.service >/dev/null <<EOF
[Unit]
Description=Digital-twin PTP grandmaster (ptp4l on ${iface})
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
ExecStart=${ptp4l_bin} -f ${conf} -i ${iface} -m
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

if [ "$ts_mode" = "hardware" ]; then
  # System clock (NTP-disciplined wall time) is the source; the NIC PHC follows it.
  # -O 37: PHC carries TAI (PTP timescale); slaves subtract currentUtcOffset (37 s) to get UTC.
  # Verified with the RoboSense Airy (PTP-E2E): with -O 0 its stamps were exactly 37 s behind.
  sudo tee /etc/systemd/system/dt-phc2sys.service >/dev/null <<EOF
[Unit]
Description=Digital-twin phc2sys (CLOCK_REALTIME -> ${iface} PHC)
After=dt-ptp4l.service
Requires=dt-ptp4l.service

[Service]
Type=simple
ExecStart=${phc2sys_bin} -s CLOCK_REALTIME -c ${iface} -O 37 -S 1 -m
Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF
else
  sudo systemctl disable --now dt-phc2sys.service 2>/dev/null || true
  sudo rm -f /etc/systemd/system/dt-phc2sys.service
  log "software timestamping: phc2sys not needed"
fi

sudo systemctl daemon-reload
sudo systemctl enable dt-ptp4l.service && sudo systemctl restart dt-ptp4l.service
if [ "$ts_mode" = "hardware" ]; then
  sudo systemctl enable --now dt-phc2sys.service
fi
sleep 2
systemctl --no-pager --lines=5 status dt-ptp4l.service || true

cat <<EOF

Verify:
  journalctl -u dt-ptp4l -f                    # expect: "assuming the grand master role"
  sudo pmc -u -b 0 'GET PORT_DATA_SET'         # portState MASTER
  sudo pmc -u -b 0 'GET TIME_STATUS_NP'
  ros2 topic echo /livox/lidar --field header.stamp --once    # compare with: date +%s.%N
  ros2 run dt_tools dt_monitor                 # reports since-boot stamps / offsets

Manual step for the RoboSense Airy (once):
  Open http://192.168.1.200 -> Setting -> Time Synchronization Source = PTP-E2E (L4),
  domain 0, save, power-cycle. The status page should show Time Sync: Lock.
  Keep use_lidar_clock: true in the rslidar config.

The Mid-360 follows the PTP master automatically (IEEE 1588v2, UDPv4, E2E).
EOF
