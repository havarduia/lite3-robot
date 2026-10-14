#!/bin/bash
# Copy one floor map from the Orin to the perception computer. Run on the laptop.
#
#   env/get_map.sh <run folder under the Orin's ~/maps> <floorN> <floor name>
#   env/get_map.sh                     list the Orin's runs that have a floor map
#
# Lands as ~/lite3_maps/<floor name>/map.{pgm,yaml}; places.json is left alone,
# and the map it replaces is kept as map.pgm.prev / map.yaml.prev.
# On the way, for a run on one level: where the lidar saw floor is floor
# (floor_free.py), and so is where he walked (walked_free.py).
#
# Over the robot's own Wi-Fi (handheld mode, no Tailscale):
#   ROBOT=ysc@192.168.1.103 JUMP=ysc@192.168.2.1 env/get_map.sh ...
set -e
ROBOT=${ROBOT:-ysc@lite3-perception}
if [ -n "$JUMP" ]; then
    TO_ROBOT="ssh -o BatchMode=yes -J $JUMP $ROBOT"
    ORIN="ssh -o BatchMode=yes -J $JUMP lite3@192.168.1.5"
else
    TO_ROBOT="ssh -o BatchMode=yes $ROBOT"
    ORIN="ssh -o BatchMode=yes -J $ROBOT lite3@192.168.1.5"
fi
if [ $# -ne 3 ]; then
    $ORIN 'cd ~/maps && ls -dt */slam_output/latest/walls/floor*.yaml' | sed 's|/slam_output/latest/walls/| |; s|\.yaml$||'
    exit 0
fi
case "$1$2$3" in *[!A-Za-z0-9._-]*) echo "error: letters, digits, . _ - only" >&2; exit 1 ;; esac
SRC="maps/$1/slam_output/latest/walls/$2"
DST="lite3_maps/$3"
$TO_ROBOT "mkdir -p $DST && for f in map.pgm map.yaml; do [ -f $DST/\$f ] && cp -p $DST/\$f $DST/\$f.prev; done; true"
# pipefail: a missing map must not leave an empty file that looks like one
set -o pipefail
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
$ORIN "cat $SRC.pgm" > "$TMP/map.pgm"
$ORIN "cat $SRC.yaml" | sed 's|^image:.*|image: map.pgm|' > "$TMP/map.yaml"
# Only for a run on one level: the path and the floor under it are one storey's.
if [ "$($ORIN "ls maps/$1/slam_output/latest/walls/" | grep -c 'yaml$')" = 1 ]; then
    $ORIN "cat maps/$1/slam_output/latest/gravity_aligned_path.npy" > "$TMP/path.npy"
    $ORIN "cat maps/$1/slam_output/latest/gravity_aligned.ply" > "$TMP/cloud.ply"
    python3 "$(dirname "$0")/floor_free.py" "$TMP/map.pgm" "$TMP/map.yaml" "$TMP/cloud.ply" "$TMP/path.npy"
    python3 "$(dirname "$0")/walked_free.py" "$TMP/map.pgm" "$TMP/map.yaml" "$TMP/path.npy"
fi
$TO_ROBOT "cat > $DST/map.pgm" < "$TMP/map.pgm"
$TO_ROBOT "cat > $DST/map.yaml" < "$TMP/map.yaml"
$TO_ROBOT "sync; ls -la $DST && cat $DST/map.yaml"
