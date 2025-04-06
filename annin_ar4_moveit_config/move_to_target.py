#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from moveit_commander import MoveGroupCommander, PlanningSceneInterface, RobotCommander
from geometry_msgs.msg import Pose

def main():
    rclpy.init()

    robot = RobotCommander()
    scene = PlanningSceneInterface()
    group = MoveGroupCommander("ar_manipulator")  # グループ名は urdf や SRDF に合わせる

    # ターゲット姿勢を設定
    target_pose = Pose()
    target_pose.position.x = 0.3
    target_pose.position.y = 0.2
    target_pose.position.z = 0.4
    target_pose.orientation.w = 1.0

    group.set_pose_target(target_pose)
    plan = group.plan()
    group.execute(plan[1], wait=True)

    rclpy.shutdown()

if __name__ == '__main__':
    main()
