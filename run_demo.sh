#!/usr/bin/env bash
# One-command Fleetra: Multi-Agent Autonomous Warehouse Logistics Orchestration Demo.
#   1. xhost + start (or reuse) the container   2. launch the world (documented launch file)
#   3. wait until both Nav2 stacks are up         4. Nav2 Startup + initial poses (no RViz clicks)
#   5. run src/exercise/multi-robot.py (create_test_plan)
#   Ctrl+C -> clean shutdown + xhost -local:docker
set -u
DEMO_DIR="$(cd "$(dirname "$0")" && pwd)"
IMAGE="${IMAGE:-fleetra:latest}"
if ! docker image inspect "$IMAGE" >/dev/null 2>&1; then
  IMAGE="jderobot/robotics-backend-ros2:amazon-warehouse-multi-robot"
fi
NAME=fleetra
XSOCK=/tmp/.X11-unix
ROBOTS="robot1 robot2"
LOG_DIR="$DEMO_DIR/logs"; mkdir -p "$LOG_DIR"
LAUNCH_LOG="$LOG_DIR/launch_$(date +%Y%m%d_%H%M%S).log"
ROS="cd /opt/warehouse_ws && . ./install/setup.sh"
T0=$(date +%s)
say() { printf '\e[1;36m[fleetra %4ss]\e[0m %s\n' "$(( $(date +%s) - T0 ))" "$*"; }
in_ctr() { docker exec "$NAME" bash -c "$ROS && $*"; }

# only one demo at a time (a second copy's cleanup would kill the first one's sim)
exec 9>"${XDG_RUNTIME_DIR:-/tmp}/fleetra.lock"
flock -n 9 || { echo "Fleetra demo is already running. Stop it with Ctrl+C or ./stop_demo.sh"; exit 1; }

cleanup() {
  trap - INT TERM EXIT
  say "Shutting down..."
  docker exec "$NAME" bash -c 'pkill -INT -f multi-robot.py; pkill -INT -f "ros2 launch"' 2>/dev/null
  sleep 5
  docker exec "$NAME" bash -c 'pkill -9 -f "gzserver|gzclient|rviz2|ros2|/opt/" ' 2>/dev/null
  docker stop -t 5 "$NAME" >/dev/null 2>&1
  xhost -local:docker >/dev/null 2>&1
  say "Stopped. (launch log: $LAUNCH_LOG)"
  exit 0
}
trap cleanup INT TERM EXIT

# ---------- 1. display + container ----------
xhost +local:docker >/dev/null
touch "$HOME/.Xauthority"
GPU_ARGS=(--device /dev/dri)                       # Intel/Mesa (this PC)
if command -v nvidia-smi >/dev/null && nvidia-smi >/dev/null 2>&1 \
   && docker info 2>/dev/null | grep -qi nvidia; then GPU_ARGS=(--gpus all); fi

if docker ps -a --format '{{.Names}}' | grep -qx "$NAME"; then
  say "Reusing container $NAME (restarting it for a clean state)"
  docker restart -t 3 "$NAME" >/dev/null
else
  say "Creating container $NAME"
  docker run -d --init --name "$NAME" --net=host --privileged --shm-size=1g "${GPU_ARGS[@]}" \
    -e DISPLAY="$DISPLAY" -e QT_X11_NO_MITSHM=1 -e CYCLONEDDS_URI=file:///demo_scripts/cyclonedds_localhost.xml \
    -e GAZEBO_MODEL_PATH=/opt/warehouse_ws/src/amazon_robot_gazebo/models:/opt/warehouse_ws/src/aws-robomaker-small-warehouse-world/models \
    -v "$XSOCK:$XSOCK:rw" -v "$HOME/.Xauthority:/root/.Xauthority:ro" \
    -v "$DEMO_DIR/scripts:/demo_scripts:ro" -v "$DEMO_DIR/params:/demo_params:ro" \
    "$IMAGE" sleep infinity >/dev/null || { say "docker run failed"; exit 1; }
fi

# ---------- 2. world (documented launch) ----------
# robot spawner with the "1"/"2" number labels (host copy -> installed python module)
docker cp "$DEMO_DIR/CustomRobots/amazon_robot/amazon_robot_bringup/amazon_robot_gazebo_spawner/amazon_robot_gazebo_spawner/amazon_robot_gazebo_spawner.py" \
  "$NAME:/opt/warehouse_ws/src/amazon_robot_bringup/amazon_robot_gazebo_spawner/amazon_robot_gazebo_spawner/amazon_robot_gazebo_spawner.py" >/dev/null
say "Launching Gazebo warehouse + 2 robots + 2 RViz (log: $LAUNCH_LOG)"
docker exec "$NAME" bash -c "$ROS && exec ros2 launch amazon_robot_bringup aws_amazon_robot_multiple.py robot1_params_file:=/demo_params/nav2_multirobot_params_1_with_control.yaml robot2_params_file:=/demo_params/nav2_multirobot_params_2_with_control.yaml map:=/demo_params/map_3beds/map.yaml" \
  > "$LAUNCH_LOG" 2>&1 &

# ---------- 3. wait for both robots + Nav2 lifecycle managers ----------
wait_for() {  # wait_for <timeout> <description> <cmd>
  local end=$(( $(date +%s) + $1 ))
  until eval "$3" >/dev/null 2>&1; do
    [ "$(date +%s)" -gt "$end" ] && { say "TIMEOUT waiting for $2"; exit 1; }
    sleep 2
  done; say "$2: ok"
}
wait_for 240 "both robots spawned in Gazebo" \
  '[ "$(grep -c "Successfully spawned entity" "$LAUNCH_LOG")" -ge 2 ]'
# only 3 beds, every other slot (3.7 m apart so a turning bed never hits its neighbour):
# remove Gazebo racks at y -1.2, -4.8, -8.7 (also erased in map_3beds)
for rack in pallet_3 pallet_1 pallet_2; do
  wait_for 30 "removed rack $rack" \
    "in_ctr 'timeout 10 ros2 service call /delete_entity gazebo_msgs/srv/DeleteEntity \"{name: $rack}\" | grep -Eq \"success=True|not exist\"'"
done
wait_for 240 "Nav2 lifecycle managers up" \
  "[ \"\$(in_ctr 'ros2 service list' | grep -c lifecycle_manager_.*/manage_nodes)\" -ge 4 ]"

# lidar box filter -> /robotN/scan_filtered (hides the legs of a carried rack)
docker exec -d "$NAME" bash -c "$ROS && exec python3 /demo_scripts/rack_scan_filter.py $ROBOTS > /tmp/rack_scan_filter.log 2>&1"
wait_for 60 "lidar box filters publishing" \
  "in_ctr 'python3 /demo_scripts/count_msgs.py /robot1/scan_filtered 3 && python3 /demo_scripts/count_msgs.py /robot2/scan_filtered 3'"

# ---------- 4. replace the manual RViz steps ----------
say "Nav2 Startup: localization"
in_ctr "python3 /demo_scripts/activate_nav2.py --stage localization $ROBOTS" || exit 1
say "Setting initial poses"
in_ctr "python3 /demo_scripts/set_initial_poses.py" || say "WARNING: pose check not perfect, continuing"
say "Nav2 Startup: navigation"
in_ctr "python3 /demo_scripts/activate_nav2.py --stage navigation $ROBOTS" || exit 1

# ---------- 5. the documented plan ----------
say "Running src/exercise/multi-robot.py (create_test_plan). Ctrl+C to stop."
docker cp "$DEMO_DIR/CustomRobots/amazon_robot/exercise/multi-robot.py" "$NAME:/opt/warehouse_ws/src/exercise/multi-robot.py"
in_ctr "exec python3 src/exercise/multi-robot.py" 2>&1 | tee "$LOG_DIR/plan.log"
wait
