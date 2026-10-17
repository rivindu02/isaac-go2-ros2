#!/bin/bash

# Get the directory of the script
DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
cd $DIR

# --gui (any position): show the Go2 sim in a window on the server display instead of WebRTC
ARGS=()
ISAAC_GUI=0
for arg in "$@"; do
    if [[ "$arg" == "--gui" ]]; then ISAAC_GUI=1; else ARGS+=("$arg"); fi
done
set -- "${ARGS[@]}"
export ISAAC_GUI

# Windows go to the server display (:0, GPU2 dummy plug) unless DISPLAY is already set
export DISPLAY=${DISPLAY:-:0}

# Open Xhost for Isaac Sim GUI
xhost +local: > /dev/null 2>&1

cd ./docker

# Environment variables
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:=100}

# Create or overwrite the .env file with static settings
cat <<EOL > .env
# Docker Compose Settings
COMPOSE_PROJECT_NAME=isaac-go2-ros2
COMPOSE_BAKE=true

# ROS Settings
ROS_DISTRO=humble

# Dynamic UID and GID
# LOCAL_USER_ID=$(id -u)
# LOCAL_GROUP_ID=$(id -g)
LOCAL_USER_ID=1000
LOCAL_GROUP_ID=1000
EOL

# Ensure cleanup on script exit
cleanup() {
    if [[ "$ACTION" != "close" && "$ACTION" != "build" && "$ACTION" != "run" && "$ACTION" != "sim" ]]; then
        if docker ps --filter "name=isaac-go2-ros2" --filter "status=running" | grep -q "isaac-go2-ros2"; then
            echo "Container is still running. Use 'close' to stop it."
        fi
    elif [[ "$ACTION" == "close" ]]; then
        if docker ps --filter "name=isaac-go2-ros2" --filter "status=running" | grep -q "isaac-go2-ros2"; then
            docker compose down
        fi
    fi
    xhost -local: > /dev/null 2>&1
}
trap cleanup EXIT

# Argument handling
ACTION=${1:-}
CUSTOM_COMMAND=${2:-}

print_usage() {
    echo -e "\033[1;32m----- [ isaac-launch Usage ] ----------------\033[0m"
    echo -e "\033[1;32m|\033[0m   build           - Build the ROS workspace"
    echo -e "\033[1;32m|\033[0m   run             - Run Isaac Go2 simulation"
    echo -e "\033[1;32m|\033[0m   sim             - Launch Isaac Sim GUI"
    echo -e "\033[1;32m|\033[0m   enter           - Enter the container"
    echo -e "\033[1;32m|\033[0m   close           - Stop the container"
    echo -e "\033[1;32m|\033[0m   --command <cmd> - Run a custom command"
    echo -e "\033[1;32m|\033[0m   --gui           - With run: sim window on \$DISPLAY (default :0), no WebRTC"
    echo -e "\033[1;32m---------------------------------------------\033[0m"
}

if [[ "$ACTION" == "--command" ]]; then
    if [[ -z "$CUSTOM_COMMAND" ]]; then
        echo "Error: No command specified for --command."
        exit 1
    fi
    if ! docker ps --filter "name=isaac-go2-ros2" --filter "status=running" | grep -q "isaac-go2-ros2"; then
        echo "Container 'isaac-go2-ros2' is not running. Starting it..."
        docker compose up -d
    fi
    docker compose exec isaac-go2-ros2 bash -c "$CUSTOM_COMMAND"
    exit 0
fi

case $ACTION in
    build)
        export AUTO_BUILD=true AUTO_RUN=false
        echo "Building ROS workspace..."
        docker compose up
        ;;
    run)
        export AUTO_BUILD=false AUTO_RUN=true
        if [[ "$ISAAC_GUI" == "1" ]]; then
            echo "Running Isaac Go2 simulation (window on $DISPLAY)..."
        else
            echo "Running Isaac Go2 simulation (WebRTC livestream)..."
        fi
        docker compose up
        docker compose down
        ;;
    sim)
        export AUTO_BUILD=false AUTO_RUN=false
        echo "Launching Isaac Sim GUI on $DISPLAY..."
        # Reuse a running container so a sim already in it keeps running
        WAS_RUNNING=false
        if docker ps --filter "name=isaac-go2-ros2" --filter "status=running" | grep -q "isaac-go2-ros2"; then
            WAS_RUNNING=true
        else
            docker compose up -d
        fi
        # Isaac Sim's bundled ROS 2 libs have no CycloneDDS, so its bridge needs Fast DDS
        docker exec -it -e DISPLAY="$DISPLAY" -e RMW_IMPLEMENTATION=rmw_fastrtps_cpp \
            isaac-go2-ros2 bash -c "/isaac-sim/isaac-sim.sh"
        if [[ "$WAS_RUNNING" == "false" ]]; then
            echo "Isaac Sim process exited. Stopping the container..."
            docker compose down
        else
            echo "Isaac Sim process exited. Container was already running, leaving it up."
        fi
        ;;
    enter)
        export AUTO_BUILD=false AUTO_RUN=false
        echo "Entering the container..."
        if ! docker ps --filter "name=isaac-go2-ros2" --filter "status=running" | grep -q "isaac-go2-ros2"; then
            echo -e "Container 'isaac-go2-ros2' is not running.\nStarting without build or run..."
            docker compose up -d
        fi
        docker exec -it -e DISPLAY="$DISPLAY" --user ubuntu isaac-go2-ros2 bash || echo "Exited container without shutting it down."
        ;;
    close)
        export AUTO_BUILD=false AUTO_RUN=false
        if docker ps --filter "name=isaac-go2-ros2" --filter "status=running" | grep -q "isaac-go2-ros2" || \
           docker ps --filter "name=isaac-go2-ros2" --filter "status=exited" | grep -q "isaac-go2-ros2"; then
            echo "Stopping the container..."
            docker compose down
        else
            echo "Container is already stopped."
        fi
        ;;
    *)
        print_usage
        exit 1
        ;;
esac
