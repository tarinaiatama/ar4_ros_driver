#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, PositionConstraint, OrientationConstraint
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

        # MoveIt用のリクエストを作成
        req = MotionPlanRequest()
        req.group_name = 'ar_manipulator'
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        # ゴール姿勢を定義
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.pose.position.x = -0.007
        pose.pose.position.y = -0.328
        pose.pose.position.z = 0.475
        pose.pose.orientation.x = 0.0
        pose.pose.orientation.y = 0.0
        pose.pose.orientation.z = 0.0
        pose.pose.orientation.w = 1.0

        # 姿勢制約
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = pose.header.frame_id
        position_constraint.link_name = 'ee_link'
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        # ボックスで制約領域を定義
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.01, 0.01, 0.01]

        position_constraint.constraint_region.primitives.append(box)
        position_constraint.constraint_region.primitive_poses.append(pose.pose)

        # 方向性制約
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = pose.header.frame_id
        orientation_constraint.link_name = 'ee_link'
        orientation_constraint.orientation = pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.1
        orientation_constraint.absolute_y_axis_tolerance = 0.1
        orientation_constraint.absolute_z_axis_tolerance = 0.1
        orientation_constraint.weight = 1.0

        goal_constraints = Constraints()
        goal_constraints.position_constraints.append(position_constraint)
        goal_constraints.orientation_constraints.append(orientation_constraint)

        req.goal_constraints.append(goal_constraints)
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
