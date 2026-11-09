#!/bin/bash
# The ORIN's campus Wi-Fi (student-FoU): reconnect when the gateway stops answering, and
# roam to a stronger access point before the current one dies (docs/panel.md, "The robot's
# internet"). Installed as /usr/local/bin/wifi-watchdog.sh, run by env/wifi-watchdog.service.
#
# Test round 2026-10-19, then on eduroam (the late roaming is the Orin's driver, not the
# network): of 5 access point changes in 13 min only 1 was a clean hand-over. The rest clung to the old access point until the gateway went quiet,
# then were down 13-35 s. The driver roams late, and wpa_supplicant's bgscan only
# scans every 30 s below -65 dBm. `wpa_cli roam` moves in ~1 s with no loss.
IF=${IF:-wlP1p1s0}
CON=${CON:-student-FoU}     # NetworkManager connection, and the SSID to roam within
EVERY=${EVERY:-2}            # s between gateway pings
MISSES=${MISSES:-3}          # missed pings in a row before a full reconnect (~6-9 s)
ROAM_DBM=${ROAM_DBM:--70}    # weaker than this: look for a better access point
ROAM_GAIN=${ROAM_GAIN:-8}    # and move only to one this many dB stronger
SCAN_EVERY=${SCAN_EVERY:-10} # s between those scans while weak

wpa() { wpa_cli -p /run/wpa_supplicant -i "$IF" "$@"; }

n=0; t=$SCAN_EVERY
while sleep "$EVERY"; do
  # Idle while the Wi-Fi is off on purpose.
  nmcli -t -f GENERAL.STATE con show "$CON" 2>/dev/null | grep -q activated || { n=0; continue; }
  gw=$(ip route show default dev "$IF" | awk '{print $3; exit}')
  if [ -n "$gw" ] && ping -c1 -W1 -I "$IF" "$gw" >/dev/null 2>&1; then n=0; else n=$((n+1)); fi
  if [ "$n" -ge "$MISSES" ]; then
    # After a drop the link can come back without a fresh DHCP lease, and the campus
    # network then blocked our IPv4 (6 min on eduroam, 2026-10-15). A full reconnect gets one.
    echo "no gateway for $n pings: reconnecting $CON"
    nmcli con up "$CON"; n=0; sleep 20; continue
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
    wpa roam "$2" >/dev/null
  fi
done
