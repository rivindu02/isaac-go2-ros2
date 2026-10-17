#!/usr/bin/env bash
# List or re-place movable objects in the running Isaac Sim scene.
# Run on the entc host while the sim container is up with object_mover.enable: True.
#
#   ./sim_objects.sh list [--label forklift] [--save]   # list_sim_objects.py
#   ./sim_objects.sh move [--random 3] [--move ID=...] [--reset] [--dry-run]   # move_sim_objects.py
#
# See the docstrings of list_sim_objects.py / move_sim_objects.py (--help) for all options.
set -euo pipefail

case "${1:-}" in
    list) SCRIPT=list_sim_objects.py ;;
    move) SCRIPT=move_sim_objects.py ;;
    *) echo "usage: $0 list|move [args...]"; exit 1 ;;
esac
shift

TTY=$([[ -t 0 ]] && echo -it || echo -i)
ARGS=""
(( $# )) && ARGS=$(printf '%q ' "$@")   # printf with no args would emit '' (one empty argument)
# runs as root in the container; hand the spot file and logs back to the host user after
OWNER="$(id -u):$(id -g)"
docker exec $TTY isaac-go2-ros2 bash -lc \
  "source /opt/ros/humble/setup.bash; cd /home/ubuntu/isaac-go2-ros2; /usr/bin/python3 -u $SCRIPT $ARGS; rc=\$?; \
   chown -R $OWNER logs cfg/object_spots.yaml 2>/dev/null; exit \$rc"
