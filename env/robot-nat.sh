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
