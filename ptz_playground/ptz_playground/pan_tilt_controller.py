#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import JointState

# 카메라 스펙 한계값 (degrees → radians)
PAN_LIMIT      = math.radians(170)   # ±170°
TILT_UP_LIMIT  = math.radians(90)    # +90°
TILT_DOWN_LIMIT= math.radians(-20)   # -20°


class PanTiltController(Node):
    def __init__(self):
        super().__init__('pan_tilt_controller')
        #퍼블리셔: joint_states 토픽
        self.joint_pub = self.create_publisher(JointState, 'joint_states', 10)
        #구독: target_point 토픽에 PointStamped 형태로 좌표 입력
        self.target_sub = self.create_subscription(
            PointStamped,
            'target_point',
            self.target_callback,
            10
        )

        # 메시지 템플릿 초기화
        self.joint_state = JointState()
        self.joint_state.name     = ['pan_joint', 'tilt_joint']
        self.joint_state.velocity = [0.0, 0.0]
        self.joint_state.effort   = [0.0, 0.0]

        # URDF 상의 조인트 피벗 위치 (base_link 기준 z 높이)
        # pan_joint origin xyz="0 0 0.1"
        # tilt_joint origin xyz="0 0 0.2"
        # 0.1 + 0.2 합계 0.3m
        self.pivot_z = 0.1 + 0.2

    def target_callback(self, msg: PointStamped):
        x = msg.point.x
        y = msg.point.y
        z = msg.point.z

        # 1) 원값 계산
        pan_raw  = math.atan2(y, x)
        horiz    = math.hypot(x, y)
        tilt_raw = math.atan2(z - self.pivot_z, horiz)

        # 2) 클램핑
        pan  = max(-PAN_LIMIT, min(PAN_LIMIT, pan_raw))
        tilt = max(TILT_DOWN_LIMIT, min(TILT_UP_LIMIT, tilt_raw))

        # 3) 불가능 판별
        if abs(pan - pan_raw) > 1e-6 or abs(tilt - tilt_raw) > 1e-6:
            self.get_logger().error(
                f"수행 불가: 목표가 물리 범위를 벗어남."
                f"pan_raw={pan_raw:.2f}, tilt_raw={tilt_raw:.2f}"
            )
            return  # 퍼블리시 생략

        # 4) 정상 퍼블리시
        now = self.get_clock().now().to_msg()
        self.joint_state.header.stamp = now
        self.joint_state.position     = [pan, tilt]
        self.joint_pub.publish(self.joint_state)
        self.get_logger().info(
            f"Pan: {pan:.2f} rad, Tilt: {tilt:.2f} rad → published"
        )

def main(args=None):
    rclpy.init(args=args)
    node = PanTiltController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

