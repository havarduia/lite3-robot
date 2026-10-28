#!/bin/bash
# Send a floor map (and the navigation bundle) from the Jetson to the robot PC.
#
#   robot-send ~/maps/<walk>/slam_output/latest/walls/floor1.yaml
#   robot-send --setup        once: copy this Jetson's SSH key to the robot PC
#   robot-send --check        is the robot PC reachable?
#
# Robot PC address/user live in ~/robot_pc.env. The bundle (~/robot_pc: Nav2
# params, launch, web bridge, README) is rsynced each time, so the robot PC
# always runs the configs kept here. The map lands as
# ~/lite3_nav/maps/<walk>_floorN.{pgm,yaml}, and maps/current.yaml points at it.
set -e
ENV="$HOME/robot_pc.env"
[ -f "$ENV" ] || { echo "error: $ENV missing" >&2; exit 1; }
source "$ENV"
[ -n "$ROBOT_PC_HOST" ] && [ -n "$ROBOT_PC_USER" ] || {
    echo "error: set ROBOT_PC_HOST and ROBOT_PC_USER in $ENV" >&2; exit 1; }
DEST="$ROBOT_PC_USER@$ROBOT_PC_HOST"
DIR="${ROBOT_PC_DIR:-lite3_nav}"
SSH=(ssh -o BatchMode=yes -o ConnectTimeout=5)

case "$1" in
    --setup)
        [ -f ~/.ssh/id_ed25519 ] || ssh-keygen -t ed25519 -N "" -f ~/.ssh/id_ed25519
        exec ssh-copy-id "$DEST" ;;
    --check)
        if "${SSH[@]}" "$DEST" true 2>/dev/null; then echo "robot PC reachable ($DEST)"
        else echo "robot PC NOT reachable ($DEST)"; exit 1; fi
        exit 0 ;;
esac

YAML=$(realpath "${1:?usage: robot-send <floorN.yaml> | --setup | --check}")
[ -f "$YAML" ] || { echo "error: no such map: $YAML" >&2; exit 1; }
WALK=$(basename "$(realpath "$(dirname "$YAML")/../../..")")
NAME="${WALK}_$(basename "$YAML" .yaml)"

"${SSH[@]}" "$DEST" true 2>/dev/null || {
    echo "error: robot PC not reachable ($DEST) - is it connected? robot-send --check" >&2
    exit 1; }

echo "bundle -> $DEST:$DIR/"
"${SSH[@]}" "$DEST" "mkdir -p $DIR/maps"
rsync -a -e "${SSH[*]}" "$HOME/robot_pc/" "$DEST:$DIR/"

echo "map    -> $DEST:$DIR/maps/$NAME.yaml"
TMP=$(mktemp -d)
cp "${YAML%.yaml}.pgm" "$TMP/$NAME.pgm"
sed "s|^image: .*|image: $NAME.pgm|" "$YAML" > "$TMP/$NAME.yaml"
rsync -a -e "${SSH[*]}" "$TMP/$NAME.pgm" "$TMP/$NAME.yaml" "$DEST:$DIR/maps/"
rm -rf "$TMP"
"${SSH[@]}" "$DEST" "cd $DIR/maps && ln -sf $NAME.yaml current.yaml"
# The remote path, for the web panel to hand to map_server.
echo "sent: $("${SSH[@]}" "$DEST" 'echo $HOME')/$DIR/maps/$NAME.yaml"
