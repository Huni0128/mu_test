#!/usr/bin/env python3
"""
LaserScan 부분 각도 크로퍼(Node).

- 입력 LaserScan에서 지정된 각도 구간만 잘라서 퍼블리시합니다.
- 각도 단위는 degrees/radians 중 선택 가능(기본: degrees).
- 인덱스 경계 계산 시 포함 구간 [start, end] 로 처리합니다.
"""

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class LaserScanCrop(Node):
    """LaserScan을 주어진 각도 범위로 잘라내어 퍼블리시하는 ROS2 노드"""

    def __init__(self):
        super().__init__('laser_scan_crop')

        # ---- 파라미터 선언 ----
        # topic 파라미터
        self.declare_parameter('input',  'scan')          # 입력 토픽명
        self.declare_parameter('output', 'scan_cropped')  # 출력 토픽명

        # 크롭 각도 파라미터
        self.declare_parameter('lower', -90.0)            # 하한 각도(기본 deg)
        self.declare_parameter('upper',  90.0)            # 상한 각도(기본 deg)
        self.declare_parameter('use_degrees', True)       # True: deg, False: rad

        # ---- 퍼블리셔/서브스크라이버 생성 ----
        in_topic  = self.get_parameter('input').value
        out_topic = self.get_parameter('output').value
        self.pub = self.create_publisher(LaserScan, out_topic, 10)
        self.sub = self.create_subscription(LaserScan, in_topic, self.cb, 10)

        self.get_logger().info(f'Cropping {in_topic} -> {out_topic}')

    def _get_crop_bounds_rad(self):
        """
        파라미터에서 크롭 경계를 읽어 라디안(rad) 단위로 반환합니다.

        Returns:
            Tuple[float, float]: (lower_rad, upper_rad)
        """
        lower = float(self.get_parameter('lower').value)
        upper = float(self.get_parameter('upper').value)
        use_deg = bool(self.get_parameter('use_degrees').value)
        if use_deg:
            lower = math.radians(lower)
            upper = math.radians(upper)
        return lower, upper

    def cb(self, msg: LaserScan):
        """
        LaserScan 콜백: 각도 범위를 계산하여 부분 시퀀스만 퍼블리시합니다.

        처리 순서:
        1) 파라미터 기준 크롭 범위(라디안) 계산
        2) 입력 메시지의 angle_min/max와 증분(angle_increment)로 인덱스 변환
        3) 유효성 검사 후 새로운 LaserScan 생성 및 publish
        """
        lower, upper = self._get_crop_bounds_rad()

        # 유효 범위 확인
        if not (upper > lower):
            self.get_logger().warn('upper must be > lower')
            return

        amin, amax, ainc = msg.angle_min, msg.angle_max, msg.angle_increment
        N = len(msg.ranges)
        if N == 0 or ainc <= 0.0:
            # 빈 스캔 또는 각 증분이 비정상인 경우
            return

        # ---- 요청 경계를 입력 스캔 경계로 클리핑 ----
        l = max(lower, amin)
        u = min(upper, amax)

        # ---- 각도 → 인덱스 변환 (포함 구간 [start, end]) ----
        start = max(0, int(math.ceil((l - amin) / ainc)))
        end   = min(N - 1, int(math.floor((u - amin) / ainc)))
        if end <= start:
            self.get_logger().warn('Crop range produces empty scan')
            return

        # ---- 출력 메시지 구성 ----
        out = LaserScan()
        out.header = msg.header
        out.angle_min = amin + start * ainc
        out.angle_max = amin + end   * ainc
        out.angle_increment = ainc
        out.time_increment = msg.time_increment
        out.scan_time = msg.scan_time
        out.range_min = msg.range_min
        out.range_max = msg.range_max

        # 부분 배열 슬라이싱 (포함 구간이므로 end+1)
        out.ranges = msg.ranges[start:end+1]
        if msg.intensities:
            out.intensities = msg.intensities[start:end+1]

        self.pub.publish(out)


def main():
    """ROS2 노드 실행 엔트리 포인트"""
    rclpy.init()
    node = LaserScanCrop()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
