import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from action_msgs.msg import GoalStatus

from sence_msgs.action import StaticPose

from .sence_poses import jointNames, poses, poseSec, poseNano


class PoseActionServer(Node):
    def __init__(self):
        # start the node
        super().__init__('pose_action_server')
        
        # start the pose action server
        self.pose_action_server = ActionServer(
            self,
            StaticPose,
            'static_pose',
            self.execute_callback)

        # start the joint trajectory action client
        self.jta_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        
        # print out the available poses
        self.get_logger().info('available poses: ' + str(list(poses.keys()))[1:-1])


    def execute_callback(self, goal_handle):
        self.get_logger().info('Waiting for action server...')
        self.jta_client.wait_for_server()

        self.get_logger().info('Moving to pose: ' + goal_handle.request.pose)

        self.send_goal(poses[goal_handle.request.pose])
  
        goal_handle.succeed()
        result = StaticPose.Result()
        result.success = True #todo test for success

        return result


    def send_goal(self, goal_pose):
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = self.getTrajectoryMsg(goal_pose)

        self.get_logger().info('Sending goal request...')
        self._send_goal_future = self.jta_client.send_goal_async(goal_msg)

        return


    def getTrajectoryMsg(self, pose):
        msg = JointTrajectory()
        msg.joint_names = jointNames
        point1 = JointTrajectoryPoint()
        point1.positions = pose
        point1.time_from_start.sec = poseSec
        point1.time_from_start.nanosec = poseNano # .5 second
        msg.points.append(point1)
        return msg


def main(args=None):
    rclpy.init(args=args)

    pose_action_server = PoseActionServer()

    rclpy.spin(pose_action_server)


if __name__ == '__main__':
    main()