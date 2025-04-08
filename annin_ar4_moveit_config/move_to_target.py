#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit2 import MoveIt2
from moveit2.robots import get_move_group_interface

class MoveToTarget(Node):
    def __init__(self):
        super().__init__('move_to_target')

        # MoveIt2の初期化
        self.moveit2 = MoveIt2(
            node=self,
            joint_names=[
                "joint_1", "joint_2", "joint_3",
                "joint_4", "joint_5", "joint_6"
            ],
            base_link_name="base_link",
            end_effector_name="tool0",
            group_name="ar_manipulator"  # ← MoveItのグループ名に合わせてください
        )

        # 目標姿勢を設定
        target_pose = PoseStamped()
        target_pose.header.frame_id = "base_link"
        target_pose.pose.position.x = 0.3
        target_pose.pose.position.y = 0.2
        target_pose.pose.position.z = 0.4
        target_pose.pose.orientation.w = 1.0

        future = self.moveit2.move_to_pose(target_pose)
        rclpy.spin_until_future_complete(self, future)

        if future.result():
            self.get_logger().info("✅ Successfully moved to target!")
        else:
            self.get_logger().error("❌ Failed to move to target.")

        rclpy.shutdown()

def main():
    rclpy.init()
    move_node = MoveToTarget()

if __name__ == '__main__':
    main()
