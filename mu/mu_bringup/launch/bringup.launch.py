#!/usr/bin/env python3
"""
Bring up:
- SLLIDAR node (/scan)
- LaserScan cropper node (/scan_cropped)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    """
    LiDAR 원본과 크롭된 스캔 두 토픽만 브링업하는 ROS 2 Launch 파일.
    """
    # =========================
    # LaunchConfigurations
    # =========================
    ns = LaunchConfiguration("namespace")

    # ---- toggles ----
    enable_lidar = LaunchConfiguration("enable_lidar")
    enable_scan_crop = LaunchConfiguration("enable_scan_crop")

    # ---- SLLIDAR params ----
    lidar_channel_type = LaunchConfiguration("lidar_channel_type")
    lidar_serial_port = LaunchConfiguration("lidar_serial_port")
    lidar_serial_baudrate = LaunchConfiguration("lidar_serial_baudrate")
    lidar_frame_id = LaunchConfiguration("lidar_frame_id")
    lidar_inverted = LaunchConfiguration("lidar_inverted")
    lidar_angle_compensate = LaunchConfiguration("lidar_angle_compensate")
    lidar_scan_mode = LaunchConfiguration("lidar_scan_mode")

    # ---- LaserScan Cropper params ----
    crop_input = LaunchConfiguration("crop_input")
    crop_output = LaunchConfiguration("crop_output")
    crop_lower_deg = LaunchConfiguration("crop_lower_deg")
    crop_upper_deg = LaunchConfiguration("crop_upper_deg")
    crop_use_degrees = LaunchConfiguration("crop_use_degrees")

    base_frame = LaunchConfiguration("base_frame")
    laser_frame = LaunchConfiguration("laser_frame")

    # =========================
    # Declare Launch Arguments
    # =========================
    declared_args = [
        DeclareLaunchArgument("namespace", default_value="", description="Top-level namespace"),

        # toggles
        DeclareLaunchArgument("enable_lidar", default_value="true", description="Enable SLLIDAR node"),
        DeclareLaunchArgument("enable_scan_crop", default_value="true", description="Enable LaserScan cropper"),

        # LiDAR args
        DeclareLaunchArgument("lidar_channel_type", default_value="serial", description="serial or tcp"),
        DeclareLaunchArgument("lidar_serial_port", default_value="/dev/ttyUSB0", description="Serial device"),
        DeclareLaunchArgument("lidar_serial_baudrate", default_value="256000", description="Baudrate"),
        DeclareLaunchArgument("lidar_frame_id", default_value="laser", description="Frame ID for laser scan"),
        DeclareLaunchArgument("lidar_inverted", default_value="false", description="Invert scan data"),
        DeclareLaunchArgument("lidar_angle_compensate", default_value="true", description="Enable angle compensation"),
        DeclareLaunchArgument("lidar_scan_mode", default_value="Sensitivity", description="Scan mode"),

        # scan_crop args
        DeclareLaunchArgument("crop_input", default_value="scan", description="Crop input topic (e.g., scan)"),
        DeclareLaunchArgument("crop_output", default_value="scan_cropped", description="Crop output topic"),
        DeclareLaunchArgument("crop_lower_deg", default_value="-130.0", description="Lower angle (degrees)"),
        DeclareLaunchArgument("crop_upper_deg", default_value="130.0", description="Upper angle (degrees)"),
        DeclareLaunchArgument("crop_use_degrees", default_value="true", description="Interpret crop angles as degrees"),

        DeclareLaunchArgument("base_frame", default_value="base_link"),
        DeclareLaunchArgument("laser_frame", default_value="laser"),
    ]

    # =========================
    # Nodes
    # =========================

    # static TF: base_link -> laser
    static_tf_laser = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_tf_base_to_laser",
        arguments=["0.0", "0.0", "0.20",
                   "0.0", "0.0", "0.0", "1.0",
                   base_frame, lidar_frame_id],
        output="screen",
    )

    # LiDAR node
    lidar_node = Node(
        package="mu_bringup",
        executable="lidar_node",
        name="lidar_node",
        namespace=ns,
        output="screen",
        condition=IfCondition(enable_lidar),
        parameters=[{
            "channel_type": lidar_channel_type,
            "serial_port": lidar_serial_port,
            "serial_baudrate": lidar_serial_baudrate,
            "frame_id": lidar_frame_id,
            "inverted": lidar_inverted,
            "angle_compensate": lidar_angle_compensate,
            "scan_mode": lidar_scan_mode,
        }],
    )

    # LaserScan Cropper node
    scan_crop_node = Node(
        package="mu_bringup",
        executable="scan_crop",
        name="laser_scan_crop",
        namespace=ns,
        output="screen",
        condition=IfCondition(enable_scan_crop),
        parameters=[{
            "input": crop_input,
            "output": crop_output,
            "lower": crop_lower_deg,
            "upper": crop_upper_deg,
            "use_degrees": crop_use_degrees,
        }],
    )

    # =========================
    # LaunchDescription
    # =========================
    return LaunchDescription(
        declared_args + [
            static_tf_laser,
            lidar_node,
            scan_crop_node,
        ]
    )
