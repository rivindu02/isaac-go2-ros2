#!/usr/bin/env bash
# Walk the Go2 in Isaac Sim through the A* waypoints of a navigation plan.
#
# Run on the entc host while the sim container is up (env_name must match the
# scene the map was built from; synth_stride2_test -> full-warehouse).
#
#   ./go_to_waypoints.sh                      # forklift demo plan (synth_stride2_test)
#   ./go_to_waypoints.sh <plan.json> <scene>  # any plan from scripts/run_navigation.py
#
# Extra args after the scene are passed to waypoint_follower.py
# (e.g. --max_lin 0.3 --dry_run).
set -euo pipefail

REPO=/home/ravindu/Malshan/Scene-Graph-Navigation-with-Dynamic-Sensing
PLAN=${1:-$REPO/outputs/navigation_plans/synth_stride2_test_20260924_060244.json}
SCENE=${2:-synth_stride2_test}
shift $(( $# < 2 ? $# : 2 ))

SHARED=/home/ravindu/dataset/navigation_plans      # = /home/ubuntu/dataset/navigation_plans in container
mkdir -p "$SHARED"
cp "$PLAN" "$SHARED/"
ALIGN_SRC=$REPO/data/processed/$SCENE/gravity_alignment.json
ALIGN_ARG=""
if [[ -f $ALIGN_SRC ]]; then
    cp "$ALIGN_SRC" "$SHARED/${SCENE}_gravity_alignment.json"
    ALIGN_ARG="--alignment /home/ubuntu/dataset/navigation_plans/${SCENE}_gravity_alignment.json"
else
    echo "warning: $ALIGN_SRC not found, using identity map->sim transform"
fi

C_PLAN=/home/ubuntu/dataset/navigation_plans/$(basename "$PLAN")
TTY=$([[ -t 0 ]] && echo -it || echo -i)

docker exec $TTY isaac-go2-ros2 bash -lc \
  "source /opt/ros/humble/setup.bash; /usr/bin/python3 -u \
   /home/ubuntu/isaac-go2-ros2/waypoint_follower.py --plan $C_PLAN $ALIGN_ARG $*"
