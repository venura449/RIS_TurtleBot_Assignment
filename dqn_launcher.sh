#!/usr/bin/env bash
set -Eeuo pipefail

ROOT="/home/venu/Downloads/RIS_TurtleBot_Assignment-main"
ROS_SETUP="/opt/ros/jazzy/setup.bash"
MAP_FILE="$ROOT/maps/maze_map.yaml"
WORLD_FILE="$ROOT/worlds/maze_6x6_centered.world"
WORLD_NAME="$(sed -n 's/^[[:space:]]*<world name="\([^"]*\)".*/\1/p' "$WORLD_FILE" | head -n 1)"
PARAMS_FILE="$ROOT/config/nav2_params.yaml"
RVIZ_FILE="$ROOT/config/dqn_training.rviz"
CHECKPOINT="$ROOT/models/turtlebot3_dqn.pt"
TORCH_SITE="/home/venu/torch_venv/lib/python3.12/site-packages"

if [[ ! -f "$ROS_SETUP" || ! -f "$WORLD_FILE" || ! -f "$MAP_FILE" ]]; then
  echo "ROS Jazzy, the centered maze world, or the saved map is missing." >&2
  exit 1
fi
if [[ -z "$WORLD_NAME" ]]; then
  echo "Could not read the world name from $WORLD_FILE." >&2
  exit 1
fi

cd "$ROOT"
set +u
source "$ROS_SETUP"
source "$ROOT/install/setup.bash"
set -u
export TURTLEBOT3_MODEL=burger
export LIBGL_ALWAYS_SOFTWARE=1
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/turtlebot3_gazebo/models:${GZ_SIM_RESOURCE_PATH:-}"

mkdir -p "$HOME/.ros"
exec 9>"$HOME/.ros/dqn_maze_launcher.lock"
if ! flock -n 9; then
  echo "The DQN maze launcher is already running. Use its terminal to stop it first." >&2
  exit 1
fi
if gz service -l 2>/dev/null | grep -Fxq "/world/$WORLD_NAME/create"; then
  echo "Gazebo world '$WORLD_NAME' is already running. Stop the existing simulation before relaunching." >&2
  exit 1
fi
if timeout 4 ros2 node list 2>/dev/null | grep -Eq '^/(dqn_controller|map_server|amcl|rviz)$'; then
  echo "A DQN/AMCL/RViz session is already running. Stop it before relaunching." >&2
  exit 1
fi

LOG_DIR="$HOME/.ros/dqn_maze_$(date +%Y%m%d_%H%M%S)"
mkdir -p "$LOG_DIR"
PIDS=()

start_bg() {
  local name="$1"
  shift
  setsid "$@" >"$LOG_DIR/$name.log" 2>&1 &
  local pid=$!
  PIDS+=("$pid")
  printf 'Started %-18s pid=%s log=%s\n' "$name" "$pid" "$LOG_DIR/$name.log"
}

shutdown() {
  local result=$?
  trap - EXIT INT TERM HUP
  echo
  echo "Stopping Gazebo, ROS nodes, and RViz; DQN saves its checkpoint on shutdown..."
  local pid
  for pid in "${PIDS[@]}"; do kill -INT -- "-$pid" 2>/dev/null || true; done
  sleep 3
  for pid in "${PIDS[@]}"; do kill -TERM -- "-$pid" 2>/dev/null || true; done
  sleep 2
  for pid in "${PIDS[@]}"; do kill -KILL -- "-$pid" 2>/dev/null || true; done
  for pid in "${PIDS[@]}"; do wait "$pid" 2>/dev/null || true; done
  echo "Logs: $LOG_DIR"
  exit "$result"
}
trap shutdown EXIT
trap 'exit 130' INT TERM HUP

echo "Starting Gazebo world '$WORLD_NAME' from the centered 6x6 maze..."
start_bg gazebo gz sim -r -v 4 "$WORLD_FILE"
GAZEBO_PID="${PIDS[-1]}"

ready=0
for _ in $(seq 1 90); do
  if ! kill -0 "$GAZEBO_PID" 2>/dev/null; then
    echo "Gazebo exited before the world became ready. See $LOG_DIR/gazebo.log" >&2
    exit 1
  fi
  if gz service -l 2>/dev/null | grep -Fxq "/world/$WORLD_NAME/create"; then
    ready=1
    break
  fi
  sleep 1
done
if [[ "$ready" != 1 ]]; then
  echo "Timed out waiting for Gazebo world services. See $LOG_DIR/gazebo.log" >&2
  exit 1
fi

echo "Spawning the robot at the clear center pose (0.3, 0.3)..."
if ! ros2 run ros_gz_sim create \
  -world "$WORLD_NAME" \
  -file /opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
  -name turtlebot3_burger -x 0.3 -y 0.3 -z 0.15 \
  >"$LOG_DIR/spawn.log" 2>&1; then
  cat "$LOG_DIR/spawn.log" >&2
  exit 1
fi
cat "$LOG_DIR/spawn.log"

echo "Starting the ROS-Gazebo bridge and robot transforms..."
start_bg bridge ros2 run ros_gz_bridge parameter_bridge \
  '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
start_bg tf_base_link ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.010 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_footprint --child-frame-id base_link
start_bg tf_base_scan ros2 run tf2_ros static_transform_publisher \
  --x -0.032 --y 0 --z 0.171 --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link --child-frame-id base_scan

echo "Loading the saved map and starting AMCL/Nav2..."
start_bg nav2 ros2 launch nav2_bringup bringup_launch.py \
  "map:=$MAP_FILE" "params_file:=$PARAMS_FILE" \
  use_sim_time:=True autostart:=True
NAV2_PID="${PIDS[-1]}"

ready=0
for _ in $(seq 1 120); do
  if ! kill -0 "$NAV2_PID" 2>/dev/null; then
    echo "Nav2 exited before localization was ready. See $LOG_DIR/nav2.log" >&2
    exit 1
  fi
  map_state="$(ros2 lifecycle get /map_server 2>/dev/null || true)"
  amcl_state="$(ros2 lifecycle get /amcl 2>/dev/null || true)"
  if grep -q '^active' <<<"$map_state" && grep -q '^active' <<<"$amcl_state"; then
    ready=1
    break
  fi
  sleep 2
done
if [[ "$ready" != 1 ]]; then
  echo "Timed out waiting for the saved map and AMCL. See $LOG_DIR/nav2.log" >&2
  exit 1
fi

sleep 1
echo "Setting AMCL's initial pose to the robot's known spawn position..."
initial_pose="{header: {frame_id: map}, pose: {pose: {position: {x: 0.3, y: 0.3, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}, covariance: [0.0625, 0, 0, 0, 0, 0, 0, 0.0625, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0.00469225]}}"
ros2 topic pub --once /initialpose geometry_msgs/msg/PoseWithCovarianceStamped "$initial_pose" \
  >"$LOG_DIR/initial_pose.log" 2>&1

echo "Opening RViz and resuming DQN from its checkpoint..."
start_bg rviz env MESA_GL_VERSION_OVERRIDE=3.3COMPAT \
  rviz2 -d "$RVIZ_FILE" --ros-args -p use_sim_time:=true
start_bg dqn env \
  "PYTHONPATH=$TORCH_SITE${PYTHONPATH:+:$PYTHONPATH}" \
  ros2 run dqn_navigation dqn_controller --ros-args \
  -p use_sim_time:=true -p learning:=true -p "checkpoint:=$CHECKPOINT"

echo
echo "Ready. In RViz, select Publish Point and click a reachable goal to start a DQN episode."
echo "Do not use Nav2 Goal for DQN training. Press Ctrl+C here to stop the session."
echo "Logs: $LOG_DIR"

set +e
wait -n "${PIDS[@]}"
result=$?
set -e
echo "A launched process exited; shutting down the rest of the session."
exit "$result"
