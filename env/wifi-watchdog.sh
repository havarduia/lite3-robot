#!/bin/bash
# The ORIN's campus Wi-Fi (student-FoU): reconnect when the gateway stops answering, and
# roam to a stronger access point before the current one dies (docs/panel.md, "The robot's
# internet"). Installed as /usr/local/bin/wifi-watchdog.sh, run by env/wifi-watchdog.service.
#
# Test round 2026-10-19, then on eduroam (the late roaming is the Orin's driver, not the
# network): of 5 access point changes in 13 min only 1 was a clean hand-over. The rest clung
# to the old access point until the gateway went quiet, then were down 13-35 s. The driver
# roams late, and wpa_supplicant's bgscan only scans every 30 s below -65 dBm. `wpa_cli roam`
# moves in ~1 s with no loss.
#
# 2026-11-09, student-FoU, walking into the hallway: the roam itself worked (-78 -> -58 dBm),
# but the gateway was quiet for 6 s after it (the lease being re-checked), the old MISSES=3
# tore the link down, that reconnect's scan missed the access point ("could not be found"),
# and the script then took the inactive connection for "off on purpose": 2 min offline.
IF=${IF:-wlP1p1s0}
CON=${CON:-student-FoU}     # NetworkManager connection, and the SSID to roam within
EVERY=${EVERY:-2}            # s between gateway pings
MISSES=${MISSES:-5}          # missed pings in a row before a full reconnect (~10-15 s)
ROAM_QUIET=${ROAM_QUIET:-15} # s after a roam not counted: the lease is re-checked
ROAM_DBM=${ROAM_DBM:--70}    # weaker than this: look for a better access point
ROAM_GAIN=${ROAM_GAIN:-8}    # and move only to one this many dB stronger
SCAN_EVERY=${SCAN_EVERY:-10} # s between those scans while weak

wpa() { wpa_cli -p /run/wpa_supplicant -i "$IF" "$@"; }

# Reconnect until it works: a failed `con up` leaves the connection inactive, which must not
# then read as "off on purpose". Each try rescans first (the try's own scan can miss).
reconnect() {
  until nmcli con up "$CON" >/dev/null 2>&1; do
    echo "reconnect failed, rescanning"
    nmcli dev wifi rescan ifname "$IF" 2>/dev/null; sleep 5
  done
  echo "reconnected"
}

n=0; t=$SCAN_EVERY; quiet_until=0
while sleep "$EVERY"; do
  # Idle while the Wi-Fi is off on purpose.
  nmcli -t -f GENERAL.STATE con show "$CON" 2>/dev/null | grep -q activated || { n=0; continue; }
  gw=$(ip route show default dev "$IF" | awk '{print $3; exit}')
  if [ -n "$gw" ] && ping -c1 -W1 -I "$IF" "$gw" >/dev/null 2>&1; then n=0
  elif [ "$(date +%s)" -ge "$quiet_until" ]; then n=$((n+1)); fi
  if [ "$n" -ge "$MISSES" ]; then
    # After a drop the link can come back without a fresh DHCP lease, and the campus
    # network then blocked our IPv4 (6 min on eduroam, 2026-10-15). A full reconnect gets one.
    echo "no gateway for $n pings: reconnecting $CON"
    reconnect; n=0; sleep 20; continue
  fi

  rssi=$(wpa signal_poll | sed -n 's/^RSSI=//p')
  if [ -z "$rssi" ] || [ "$rssi" -ge "$ROAM_DBM" ]; then t=$SCAN_EVERY; continue; fi
  t=$((t + EVERY)); [ "$t" -ge "$SCAN_EVERY" ] || continue; t=0
  wpa scan >/dev/null; sleep 3
  cur=$(wpa status | sed -n 's/^bssid=//p')
  # scan_results: bssid, freq, signal, flags, ssid. Strongest access point last.
  set -- $(wpa scan_results | awk -F'\t' -v ssid="$CON" '$5 == ssid {print $3, $1}' | sort -n | tail -1)
  if [ -n "$2" ] && [ "$2" != "$cur" ] && [ "$1" -ge $((rssi + ROAM_GAIN)) ]; then
    echo "roaming $cur ($rssi dBm) -> $2 ($1 dBm)"
    wpa roam "$2" >/dev/null; quiet_until=$(( $(date +%s) + ROAM_QUIET )); n=0
  fi
done
