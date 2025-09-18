-- Cartographer 구성 파일
-- - 맵/궤적 빌더 include
-- - 좌표계/센서/퍼블리시/샘플링 설정
-- - 2D 라이다 및 Pose Graph 최적화 파라미터

include "map_builder.lua"
include "trajectory_builder.lua"

options = {
  -- 빌더 객체 (2D/3D 중 선택)
  map_builder = MAP_BUILDER,
  trajectory_builder = TRAJECTORY_BUILDER,

  -- 좌표계(frame) 설정
  map_frame = "map",           -- 최종 맵 기준 좌표계
  tracking_frame = "base_link", -- 로봇 추적 좌표계(보통 IMU/센서 기준)
  published_frame = "base_link", -- 퍼블리시 좌표계(로봇 본체 기준)
  odom_frame = "odom",         -- 오도메트리 좌표계
  provide_odom_frame = true,   -- Cartographer가 odom 프레임 제공 여부
  publish_frame_projected_to_2d = true, -- 3D를 2D로 투영해 퍼블리시

  -- 센서 사용 여부
  use_odometry = false,        -- 오도메트리 사용
  use_nav_sat = false,         -- GNSS 사용
  use_landmarks = false,       -- 랜드마크 사용

  -- 센서 개수/분할
  num_laser_scans = 1,                     -- 단일 2D LiDAR: 1
  num_multi_echo_laser_scans = 0,          -- 멀티 에코 LiDAR: 없으면 0
  num_subdivisions_per_laser_scan = 1,     -- 레이저 스캔 분할 처리 개수
  num_point_clouds = 0,                    -- 3D LiDAR 포인트클라우드 센서 수

  -- 퍼블리시 주기/타임아웃
  lookup_transform_timeout_sec = 0.2,      -- TF 조회 대기 시간
  submap_publish_period_sec = 0.5,         -- 서브맵 퍼블리시 주기
  pose_publish_period_sec = 0.005,         -- 포즈 퍼블리시 주기
  trajectory_publish_period_sec = 0.03,    -- 궤적 퍼블리시 주기

  -- 샘플링 비율
  rangefinder_sampling_ratio = 1.0,        -- LiDAR 데이터 샘플링(1.0=모두 사용)
  odometry_sampling_ratio = 1.0,
  fixed_frame_pose_sampling_ratio = 1.0,   -- 고정 프레임(GPS 등)
  imu_sampling_ratio = 1.0,
  landmarks_sampling_ratio = 1.0,
}

-- 2D SLAM 모드 사용
MAP_BUILDER.use_trajectory_builder_2d = true

-- 2D 라이다 설정
TRAJECTORY_BUILDER_2D.use_imu_data = false            -- IMU 미사용
TRAJECTORY_BUILDER_2D.min_range = 0.10                -- 최소 감지 거리(m)
TRAJECTORY_BUILDER_2D.max_range = 12.0                -- 최대 감지 거리(m)
TRAJECTORY_BUILDER_2D.missing_data_ray_length = 5.0   -- 누락 시 가상 레이 길이(m)
TRAJECTORY_BUILDER_2D.num_accumulated_range_data = 1  -- 누적 스캔 수(1이면 지연 최소)

-- Pose Graph(후처리 최적화)
POSE_GRAPH.optimize_every_n_nodes = 10                         -- n개 노드마다 최적화
POSE_GRAPH.constraint_builder.min_score = 0.65                 -- 루프 클로저 최소 점수
POSE_GRAPH.constraint_builder.global_localization_min_score = 0.70
-- 글로벌 재로컬라이제이션 최소 점수

return options
