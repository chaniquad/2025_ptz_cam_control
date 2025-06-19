#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState

class PixelPanTiltController(Node):
    def __init__(self):
        super().__init__('pixel_pan_tilt_controller')

        # 1) 카메라 인스트린직 파라미터 선언 및 로드
        self.declare_parameter('fx', 600.0)
        self.declare_parameter('fy', 600.0)
        self.declare_parameter('cx', 320.0)
        self.declare_parameter('cy', 240.0)
        self.fx = self.get_parameter('fx').value
        self.fy = self.get_parameter('fy').value
        self.cx = self.get_parameter('cx').value
        self.cy = self.get_parameter('cy').value

        # 2) 퍼블리셔: pan/tilt 조인트 각도
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)

        # 3) 구독: CLI로 퍼블리시되는 픽셀 좌표 (u,v)
        self.pixel_sub = self.create_subscription(
            PointStamped,
            'target_pixel',  # ros2 topic pub /target_pixel
            self.pixel_callback,
            10
        )

        # 조인트 메시지 템플릿
        self.joint_state = JointState()
        self.joint_state.name     = ['pan_joint', 'tilt_joint']
        self.joint_state.velocity = [0.0, 0.0]
        self.joint_state.effort   = [0.0, 0.0]

    def pixel_callback(self, msg: PointStamped):
        # u, v 받아오기 (z는 무시)
        u = msg.point.x
        v = msg.point.y

        # 역투영: 카메라 좌표계 방향 벡터
        x_cam = (u - self.cx) / self.fx
        y_cam = (v - self.cy) / self.fy
        z_cam = 1.0

        # 월드 기준 방향 벡터 매핑
        dx = z_cam
        dy = -x_cam
        dz = -y_cam

        # pan/tilt 각도 계산
        pan  = math.atan2(dy, dx)
        tilt = math.atan2(dz, math.hypot(dx, dy))

        # 메시지 작성 & 발행
        now = self.get_clock().now().to_msg()
        self.joint_state.header.stamp = now
        self.joint_state.position     = [pan, tilt]
        self.joint_pub.publish(self.joint_state)

        self.get_logger().info(
            f"pixel(u={u:.1f}, v={v:.1f}) → pan={pan:.3f}, tilt={tilt:.3f}"
        )


def main(args=None):
    rclpy.init(args=args)
    node = PixelPanTiltController()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

