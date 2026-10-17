# TurtleBot3 Maze Assignment

This repository contains Gazebo maze worlds, Cartographer and Nav2 configuration, and saved map files for a TurtleBot3 Burger simulation using ROS 2 Jazzy.

## Prerequisites

- Ubuntu 24.04 under WSL, with ROS 2 Jazzy installed.
- Gazebo Sim and the TurtleBot3, `ros_gz_sim`, `ros_gz_bridge`, Cartographer, Nav2, and teleop packages installed.
- The repository available in WSL at `~/turtlebot3_assignment`.

The commands below assume the repository is at `~/turtlebot3_assignment`. If it is elsewhere, update the paths before running them. Open separate WSL terminals for the commands in each terminal section. Run the environment setup in every terminal that runs ROS commands:

Start the Ubuntu terminal from Windows Terminal or PowerShell with:

```powershell
wsl -d Ubuntu-24.04 -u venura
```

Then, in each Ubuntu terminal, run:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
```

## 1. Start the Gazebo world

In the first terminal:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger
export GZ_SIM_RESOURCE_PATH="/opt/ros/jazzy/share/turtlebot3_gazebo/models:${GZ_SIM_RESOURCE_PATH}"

gz sim -r -v 4 ~/turtlebot3_assignment/worlds/maze.world
```

The world file is [`worlds/maze.world`](worlds/maze.world). Keep this terminal running.

## 2. Spawn the TurtleBot3

In a second terminal, create the robot in the running `robot_maze` world:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 run ros_gz_sim create \
  -world robot_maze \
  -file /opt/ros/jazzy/share/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf \
  -name turtlebot3_burger \
  -x 0.25 -y 0.25 -z 0.15
```

## 3. Start the ROS-Gazebo bridge

In a third terminal:

```bash
source /opt/ros/jazzy/setup.bash

ros2 run ros_gz_bridge parameter_bridge \
  '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist' \
  '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry' \
  '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan' \
  '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock' \
  '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V'
```

Keep the bridge running while using teleoperation, mapping, or Nav2.

## 4. Publish the static robot transforms

In another terminal, start both static transform publishers:

```bash
source /opt/ros/jazzy/setup.bash

ros2 run tf2_ros static_transform_publisher \
  --x 0 --y 0 --z 0.010 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_footprint \
  --child-frame-id base_link &

ros2 run tf2_ros static_transform_publisher \
  --x -0.032 --y 0 --z 0.171 \
  --roll 0 --pitch 0 --yaw 0 \
  --frame-id base_link \
  --child-frame-id base_scan &

wait
```

Keep this terminal running.

## 5. Run teleoperation (optional)

In a separate terminal, use the keyboard to drive the robot:

```bash
source /opt/ros/jazzy/setup.bash

ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p stamped:=false
```

## 6. Run Cartographer and publish the occupancy grid

Start Cartographer in one terminal:

```bash
source /opt/ros/jazzy/setup.bash

ros2 run cartographer_ros cartographer_node \
  -configuration_directory "$HOME/turtlebot3_assignment/config" \
  -configuration_basename cartographer.lua \
  --ros-args -p use_sim_time:=true
```

In another terminal, start the occupancy grid publisher:

```bash
source /opt/ros/jazzy/setup.bash

ros2 run cartographer_ros cartographer_occupancy_grid_node \
  -resolution 0.05 \
  -publish_period_sec 1.0 \
  --ros-args -p use_sim_time:=true
```

Drive the robot around the maze so Cartographer can build the map.

## 7. View the map in RViz

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash

rviz2 --ros-args -p use_sim_time:=true
```

Configure RViz to display the map, laser scan, and TF topics as needed.

## 8. Save the map

After mapping, save the occupancy grid from a new terminal:

```bash
source /opt/ros/jazzy/setup.bash

mkdir -p ~/turtlebot3_assignment/maps

ros2 run nav2_map_server map_saver_cli \
  -f ~/turtlebot3_assignment/maps/maze_map \
  --ros-args -p use_sim_time:=true
```

This creates `maze_map.yaml` and its image file in [`maps/`](maps/).

## 9. Launch Nav2

Stop the Cartographer node and occupancy grid publisher (for example, with `Ctrl+C` in their terminals) before starting Nav2. This avoids competing localization/map transforms. Keep Gazebo, the robot, the bridge, and the static transform publishers running.

In a new terminal:

```bash
source /opt/ros/jazzy/setup.bash
export TURTLEBOT3_MODEL=burger

ros2 launch nav2_bringup bringup_launch.py \
  map:="$HOME/turtlebot3_assignment/maps/maze_map.yaml" \
  params_file:="$HOME/turtlebot3_assignment/config/nav2_params.yaml" \
  use_sim_time:=True \
  autostart:=True
```

Nav2 parameters are in [`config/nav2_params.yaml`](config/nav2_params.yaml).

## 10. Open RViz for Nav2

In another terminal:

```bash
source /opt/ros/jazzy/setup.bash

rviz2 --ros-args -p use_sim_time:=true
```

Use the **2D Pose Estimate** tool to set the robot's initial pose on the map, then use **Nav2 Goal** to send a navigation goal.

## 11. Start or activate the Nav2 lifecycle nodes

If Nav2's lifecycle nodes have not started, run this from another terminal:

```bash
source /opt/ros/jazzy/setup.bash

ros2 service call /lifecycle_manager_navigation/manage_nodes \
  nav2_msgs/srv/ManageLifecycleNodes \
  "{command: 0}"
```

Command `0` requests the lifecycle manager's startup operation. It is not a deactivation command.
