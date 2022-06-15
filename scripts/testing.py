#!/usr/bin/env python3

import time

import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from rclpy_message_converter import message_converter

class ArmClient(Node):

    def __init__(self):
        super().__init__('arm_client')
        self._action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/right_arm_joint_trajectory_controller/follow_joint_trajectory')

    def send_goal(self, positions, duration):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory.joint_names = [
          "right_ur_arm_shoulder_pan_joint",
          "right_ur_arm_shoulder_lift_joint",
          "right_ur_arm_elbow_joint",
          "right_ur_arm_wrist_1_joint",
          "right_ur_arm_wrist_2_joint",
          "right_ur_arm_wrist_3_joint",
        ]
        goal_msg.trajectory.points = [
          message_converter.convert_dictionary_to_ros_message('trajectory_msgs/JointTrajectoryPoint',
          {
            "positions": positions,
            "time_from_start": duration
          })
        ]

        self._action_client.wait_for_server()

        return self._action_client.send_goal_async(goal_msg)

def main(args=None):
    rclpy.init(args=args)

    arm_client = ArmClient()

    future = arm_client.send_goal([0.0, -0.5, 0.0, 0.0, 0.0, 0.0], [5, 0])

    rclpy.spin_until_future_complete(arm_client, future)


if __name__ == '__main__':
    main()