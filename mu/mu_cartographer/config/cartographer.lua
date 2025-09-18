include "map_builder.lua"
include "trajectory_builder.lua"

-- Cartographer SLAM 옵션 정의
options = {
  -- 기본 빌더
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- 좌표계(frame)
  map_frame = "map",
  tracking_frame = "base_link",
  published_frame = "base_link",
  odom_frame = "odom",
  provide_odom_frame = true,
  publish_frame_projected_to_2d = true,

  -- 센서 사용 여부
  use_odometry = false,      -- 외부 오도메트리
  use_nav_sat = false,       -- GNSS(GPS)
  use_landmarks = false,     -- 랜드마크

  -- 입력 데이터 종류
  num_laser_scans = 1,       -- LaserScan 토픽 개수
  num_multi_echo_laser_scans = 0,
  num_subdivisions_per_laser_scan = 1,
  num_point_clouds = 0,

  -- 샘플링 비율(Foxy에서 필수)
  rangefinder_sampling_ratio = 1.0,
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,

  -- 퍼블리시 주기/타임아웃
  lookup_transform_timeout_sec = 0.2,
  submap_publish_period_sec = 0.3,
  pose_publish_period_sec = 0.03,
  trajectory_publish_period_sec = 0.03,
}

-- Trajectory Builder
MAP_BUILDER.use_trajectory_builder_2d = true

-- LiDAR 파라미터
TRAJECTORY_BUILDER_2D.min_range = 0.15
TRAJECTORY_BUILDER_2D.max_range = 12.0
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0
TRAJECTORY_BUILDER_2D.use_imu_data = false
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1
TRAJECTORY_BUILDER_2D.voxel_filter_size = 0.025

-- 라이다 단독 매칭 안정화
TRAJECTORY_BUILDER_2D.use_online_correlative_scan_matching = true
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.linear_search_window = 0.1
TRAJECTORY_BUILDER_2D.real_time_correlative_scan_matcher.angular_search_window = math.rad(20.0)

-- Submap 및 Pose Graph
TRAJECTORY_BUILDER_2D.submaps.num_range_data = 90
POSE_GRAPH.optimization_problem.huber_scale = 1e1
POSE_GRAPH.optimize_every_n_nodes = 35
POSE_GRAPH.constraint_builder.min_score = 0.55
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.6

return options
