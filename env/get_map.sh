#!/bin/bash
# Copy one floor map from the Orin to the perception computer. Run on the laptop.
#
#   env/get_map.sh <run folder under the Orin's ~/maps> <floorN> <floor name>
#   env/get_map.sh                     list the Orin's runs that have a floor map
#
# Lands as ~/lite3_maps/<floor name>/map.{pgm,yaml}; places.json is left alone.
# The cells he walked over are made floor on the way (walked_free.py).
set -e
ROBOT=${ROBOT:-ysc@lite3-perception}
ORIN="ssh -o BatchMode=yes -J $ROBOT lite3@192.168.1.5"
if [ $# -ne 3 ]; then
    $ORIN 'cd ~/maps && ls -dt */slam_output/latest/walls/floor*.yaml' | sed 's|/slam_output/latest/walls/| |; s|\.yaml$||'
    exit 0
fi
case "$1$2$3" in *[!A-Za-z0-9._-]*) echo "error: letters, digits, . _ - only" >&2; exit 1 ;; esac
SRC="maps/$1/slam_output/latest/walls/$2"
DST="lite3_maps/$3"
ssh -o BatchMode=yes "$ROBOT" "mkdir -p $DST"
# pipefail: a missing map must not leave an empty file that looks like one
set -o pipefail
TMP=$(mktemp -d); trap 'rm -rf "$TMP"' EXIT
$ORIN "cat $SRC.pgm" > "$TMP/map.pgm"
$ORIN "cat $SRC.yaml" | sed 's|^image:.*|image: map.pgm|' > "$TMP/map.yaml"
# Where he walked is floor (walked_free.py). Only for a run on one level: the path has no floor to it.
if [ "$($ORIN "ls maps/$1/slam_output/latest/walls/" | grep -c 'yaml$')" = 1 ]; then
    $ORIN "cat maps/$1/slam_output/latest/gravity_aligned_path.npy" > "$TMP/path.npy"
    python3 "$(dirname "$0")/walked_free.py" "$TMP/map.pgm" "$TMP/map.yaml" "$TMP/path.npy"
fi
ssh -o BatchMode=yes "$ROBOT" "cat > $DST/map.pgm" < "$TMP/map.pgm"
ssh -o BatchMode=yes "$ROBOT" "cat > $DST/map.yaml" < "$TMP/map.yaml"
ssh -o BatchMode=yes "$ROBOT" "ls -la $DST && cat $DST/map.yaml"
