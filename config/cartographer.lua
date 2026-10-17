
include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  map_frame = "map",
  tracking_frame = "base_footprint",
  published_frame = "odom",
  odom_frame = "odom",

  provide_odom_frame = false,
  publish_frame_projected_to_2d = false,

  use_odometry = true,
  use_pose_extrapolator = true,
  use_nav_sat = false,
  use_landmarks = false,

  num_laser_scans = 1,
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.005,
  trajectory_publish_period_sec = 0.03,

  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

MAP_BUILDER.use_trajectory_builder_2d = true

-- No IMU required for this simulation
TRAJECTORY_BUILDER_2D.use_imu_data = false

-- LiDAR settings
TRAJECTORY_BUILDER_2D.min_range = 0.12
TRAJECTORY_BUILDER_2D.max_range = 3.5
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 3.5

-- Mapping parameters
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 2
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 45

-- Scan matching
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.translation_weight = 20
TRAJECTORY_BUILDER_2D.ceres_scan_matcher.rotation_weight = 20

-- Optimization
--POSE_GRAPH.optimize_every_n_nodes = 0
POSE_GRAPH.constraint_builder.max_constraint_distance = 5.0
POSE_GRAPH.constraint_builder.min_score = 0.9

-- Use wheel odometry for optimization
POSE_GRAPH.optimization_problem.odometry_translation_weight = 1e3
POSE_GRAPH.optimization_problem.odometry_rotation_weight = 1e3

return options
