#!/bin/bash
# deploy.sh [host] - copy orin/bin/ to the Orin's ~/bin (which is outside git). From the laptop on the
# robot's switch, or over Tailscale with the host given. The files shared with perception (lio_relay.py,
# cloud_locate.py, cloudreg.py, lidar_nav.launch.py) are links into robot/ and env/ here: -L sends the
# files themselves, -c only those that differ. Nothing is deleted or restarted on the Orin; a running
# program keeps its old code.
cd "$(dirname "$0")/bin" || exit 1
rsync -rLcv --exclude __pycache__ ./ "${1:-lite3@192.168.1.5}:bin/" && ssh "${1:-lite3@192.168.1.5}" sync
