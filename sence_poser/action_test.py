from action_msgs.msg import GoalStatus
from control_msgs.action import FollowJointTrajectory

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from .sence_poses import *

from numpy import round


class MinimalActionClient(Node):

    def __init__(self):
        super().__init__('minimal_action_client')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

        self.poseTime = 1


    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback):
        rounded_error = round (feedback.feedback.error.positions, 4)
        self.get_logger().info('Received feedback: Error = {0}'.format(rounded_error))

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded! Result: {0}'.format(result))
        else:
            self.get_logger().info('Goal failed with status: {0}'.format(status))

        # Shutdown after receiving a result
        rclpy.shutdown()

    def send_goal(self):
        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()

        goal_pose = crabStandPose

        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = self.getTrajectoryMsg(goal_pose)

        self.get_logger().info('Sending goal request...')

        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def getTrajectoryMsg(self, pose):
        msg = JointTrajectory()
        msg.joint_names = jointNames
        point1 = JointTrajectoryPoint()
        point1.positions = pose
        point1.time_from_start.sec = self.poseTime
        msg.points.append(point1)

        return msg


def main(args=None):
    rclpy.init(args=args)

    action_client = MinimalActionClient()

    action_client.send_goal()

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()