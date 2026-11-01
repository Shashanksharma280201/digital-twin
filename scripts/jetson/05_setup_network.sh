#!/usr/bin/env bash
# Static IPs for both LiDARs, large socket buffers, CycloneDDS config.
# Usage: 05_setup_network.sh [iface]
#   Mid-360 expects host 192.168.1.5 ; Airy expects host 192.168.1.102 (same /24).
set -euo pipefail
# shellcheck source=common.sh
source "$(dirname "${BASH_SOURCE[0]}")/common.sh"

step "05 network"
iface="${1:-$(detect_wired_iface || true)}"
[ -n "$iface" ] || die "Could not detect a wired interface; pass it explicitly: $0 eth0"
log "LiDAR interface: ${iface}"

have nmcli || die "nmcli (NetworkManager) not found"
con="dt-lidar"
if ! nmcli -t -f NAME connection show | grep -qx "$con"; then
  log "creating NetworkManager connection ${con} on ${iface}"
  sudo nmcli connection add type ethernet ifname "$iface" con-name "$con" \
    ipv4.method manual ipv4.addresses "192.168.1.5/24,192.168.1.102/24" \
    ipv4.never-default yes ipv6.method disabled 802-3-ethernet.mtu 1500 connection.autoconnect yes
else
  log "updating connection ${con}"
  sudo nmcli connection modify "$con" connection.interface-name "$iface" \
    ipv4.method manual ipv4.addresses "192.168.1.5/24,192.168.1.102/24" \
    ipv4.never-default yes ipv6.method disabled 802-3-ethernet.mtu 1500
fi
sudo nmcli connection up "$con" || warn "could not bring ${con} up (cable unplugged?). It will come up when linked."

step "kernel socket buffers"
# 99- so it loads after the ZED SDK's 60-zed-buffers.conf, which sets smaller limits.
sysctl_file=/etc/sysctl.d/99-dt-lidar.conf
sudo rm -f /etc/sysctl.d/60-dt-lidar.conf
sudo tee "$sysctl_file" >/dev/null <<'EOF'
# Digital-twin rig: large UDP receive buffers for LiDAR packets and DDS
net.core.rmem_max=2147483647
net.core.rmem_default=67108864
net.core.wmem_max=2147483647
net.core.wmem_default=67108864
net.ipv4.ipfrag_time=3
net.ipv4.ipfrag_high_thresh=134217728
EOF
sudo sysctl --system >/dev/null
ok "wrote ${sysctl_file}"

step "CycloneDDS config"
mkdir -p "${HOME}/.config/dt"
dds="${HOME}/.config/dt/cyclonedds.xml"
# DDS stays on loopback + the default route iface; LiDAR traffic is raw UDP and does not use DDS.
cat >"$dds" <<'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance"
            xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
  <Domain Id="any">
    <General>
      <Interfaces>
        <NetworkInterface autodetermine="true" priority="default" multicast="default" />
      </Interfaces>
      <AllowMulticast>default</AllowMulticast>
      <MaxMessageSize>65500B</MaxMessageSize>
    </General>
    <Internal>
      <SocketReceiveBufferSize min="64MB"/>
      <Watermarks>
        <WhcHigh>500kB</WhcHigh>
      </Watermarks>
    </Internal>
  </Domain>
</CycloneDDS>
EOF
ok "wrote ${dds}"

cat <<EOF

Done. env.sh exports these for you:
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  export CYCLONEDDS_URI=file://${dds}

Check the LiDARs:
  ping 192.168.1.200          # Airy (default)
  ping 192.168.1.1XX          # Mid-360: 1XX = last two digits of the serial number
EOF
