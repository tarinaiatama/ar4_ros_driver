#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint
from geometry_msgs.msg import PoseStamped
from shape_msgs.msg import SolidPrimitive

class MoveToTarget(Node):
    def __init__(self):
        super().__init__('move_to_target_client')

        self._action_client = ActionClient(self, MoveGroup, 'move_action')

        self.send_goal()

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_msg = MoveGroup.Goal()

        # motion_plan_request を設定
        req = MotionPlanRequest()
        req.group_name = 'ar_manipulator'  # ←グループ名はSRDFと一致するように

        # ターゲットの姿勢（例：位置のみ簡易設定）
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = 0.3
        pose.pose.position.y = 0.0
        pose.pose.position.z = 0.3
        pose.pose.orientation.w = 1.0

        # PositionConstraint を使用（orientationは今回は無視）
        constraint = PositionConstraint()
        constraint.header.frame_id = 'base_link'
        constraint.link_name = 'ee_link'  # ←エンドエフェクタのリンク名に合わせる
        constraint.target_point_offset.x = 0.0
        constraint.target_point_offset.y = 0.0
        constraint.target_point_offset.z = 0.0

        # ボックスとして許容範囲を設定
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]  # ほぼピンポイント

        constraint.constraint_region.primitives.append(box)
        constraint.constraint_region.primitive_poses.append(pose.pose)

        req.goal_constraints.append(Constraints(position_constraints=[constraint]))

        goal_msg.request = req

        self.get_logger().info('Sending goal...')
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Goal rejected by server.')
            return

        self.get_logger().info('✅ Goal accepted, waiting for result...')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info(f'🎉 Result received: {result.error_code}')
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MoveToTarget()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
