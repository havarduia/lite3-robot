#!/bin/sh
# On the MOTION computer, /usr/local/bin/robot-nat.sh, run at boot by robot-nat.service.
# NAT for the perception computer, plus the exemption that lets DDS traffic
# from a laptop on the AP keep its real source address (RViz discovery needs it).
iptables -t nat -C POSTROUTING -s 192.168.1.0/24 -o wlan0 -j MASQUERADE 2>/dev/null || \
  iptables -t nat -A POSTROUTING -s 192.168.1.0/24 -o wlan0 -j MASQUERADE
iptables -t nat -C POSTROUTING -s 192.168.2.0/24 -d 192.168.1.0/24 -j RETURN 2>/dev/null || \
  iptables -t nat -I POSTROUTING 1 -s 192.168.2.0/24 -d 192.168.1.0/24 -j RETURN
iptables -C FORWARD -s 192.168.1.0/24 -d 192.168.2.0/24 -j ACCEPT 2>/dev/null || \
  iptables -I FORWARD 1 -s 192.168.1.0/24 -d 192.168.2.0/24 -j ACCEPT
# The web panel for a phone on the robot's own wifi (handheld mode, no Tailscale):
# http://192.168.2.1:8080 is the panel on the perception computer, and UDP 8190 its
# camera relay (8189 here is mediamtx's own). docs/panel.md.
iptables -t nat -C PREROUTING -i p2p0 -d 192.168.2.1 -p tcp --dport 8080 -j DNAT --to-destination 192.168.1.103:8080 2>/dev/null || \
  iptables -t nat -A PREROUTING -i p2p0 -d 192.168.2.1 -p tcp --dport 8080 -j DNAT --to-destination 192.168.1.103:8080
iptables -t nat -C PREROUTING -i p2p0 -d 192.168.2.1 -p udp --dport 8190 -j DNAT --to-destination 192.168.1.103:8189 2>/dev/null || \
  iptables -t nat -A PREROUTING -i p2p0 -d 192.168.2.1 -p udp --dport 8190 -j DNAT --to-destination 192.168.1.103:8189
# The camera streams (mediamtx: RTMP 1935, RTSP 8554, HLS 8888, WebRTC 8889; RTP
# 8000-8001 and SRT 8890 over UDP) answer anyone, with no password, so they would
# skip the panel's login (PRIVACY.md). Only the perception computer (the panel,
# rs_stream.py) and the Orin (bin/run.py --frames) may reach them, and this
# computer itself over loopback: push_video.sh publishes the front camera that way,
# and loopback traffic does pass PREROUTING (2026-10-23: without it, no picture).
# Tailscale here runs in userspace mode and reaches local ports over loopback too,
# so the tailnet is shut out by `tailscale set --shields-up=true` instead (PRIVACY.md).
# In the mangle table,
# at the top: Tailscale's ts-input chain (filter) accepts all tailnet traffic
# before any filter rule, and this kernel has no raw table (CONFIG_IP_NF_RAW).
# DROP goes in first, then the ACCEPTs above it.
# The vendor phone app's camera view, if it reads these, stops working too.
for rule in "tcp 1935" "tcp 8554" "tcp 8888" "tcp 8889" "udp 8000" "udp 8001" "udp 8890"; do
  set -- $rule
  iptables -t mangle -C PREROUTING -p $1 --dport $2 -j DROP 2>/dev/null || \
    iptables -t mangle -I PREROUTING 1 -p $1 --dport $2 -j DROP
  for from in "-s 192.168.1.103" "-s 192.168.1.5" "-i lo"; do
    iptables -t mangle -C PREROUTING -p $1 --dport $2 $from -j ACCEPT 2>/dev/null || \
      iptables -t mangle -I PREROUTING 1 -p $1 --dport $2 $from -j ACCEPT
  done
done
