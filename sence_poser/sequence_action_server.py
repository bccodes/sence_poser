import rclpy
from rclpy.action import ActionServer, ActionClient
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from action_msgs.msg import GoalStatus

from sence_msgs.action import StaticPose, PoseSequence

from .sence_poses import jointNames, poses, sequences, poseSec, poseNano


class SequenceActionServer(Node):
    def __init__(self):
        # start the node
        super().__init__('sequence_action_server')

        # start the sequence action server
        self.sequence_action_server = ActionServer(
            self,
            PoseSequence,
            'pose_sequence',
            self.execute_callback)

        # start the joint trajectory action client
        self.jta_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory')
        
        # print out the available sequences
        self.get_logger().info('available sequences: ' + str(list(sequences.keys()))[1:-1])


    def execute_callback(self, goal_handle):
        self.get_logger().info('Waiting for action server...')
        self.jta_client.wait_for_server()

        self.get_logger().info('Running Sequence: ' + goal_handle.request.sequence)

        self.run_sequence(sequences[goal_handle.request.sequence])

        goal_handle.succeed()
        result = StaticPose.Result()
        result.success = True #todo test for success

        return result


    def run_sequence(self, goal_sequence):
        goal_msg = FollowJointTrajectory.Goal()

        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = jointNames

        
        for (index, pose) in list(enumerate(sequences[goal_sequence])):
            self.get_logger().info('adding trajectory point ' + index + " " "data " + pose)
            point = JointTrajectoryPoint()
            point.positions = pose
            point.time_from_start.sec = poseSec
            point.time_from_start.nanosec = poseNano + index * poseNano
            goal_msg.trajectory.points.append(point)

        self.get_logger().info('Sending goal request...')
        self.get_logger().info(goal_msg)

        self._send_goal_future = self.jta_client.send_goal_async(goal_msg)

        return


def main(args=None):
    rclpy.init(args=args)

    sequence_action_server = SequenceActionServer()

    rclpy.spin(sequence_action_server)


if __name__ == '__main__':
    main()