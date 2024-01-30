import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sence_msgs.action import StaticPose, PoseSequence

from .sence_poses import poses, sequences


class SenceActionMenu(Node):
    def __init__(self):
        super().__init__('sence_action_menu')

        print("""Sence Action Menu
0. Pose Client
1. Sequence Client""")

        while True:
            try:
                choice = int(input(">> "))
                match choice:
                    case 0:
                        self.poseMenu()
                    case 1:
                        self.sequenceMenu()
                    case _:
                        print("Not an option")
            except Exception as e:
                print(e)


    def poseMenu(self):
        self.pose_action_client = ActionClient(
            self,
            StaticPose,
            '/static_pose')

        self.get_logger().info('Waiting for pose server...')
        self.pose_action_client.wait_for_server()

        enumerated_poses = list(enumerate(list(poses.keys())))

        while True:
            for (index, pose) in enumerated_poses:
                print(f"{index}. {pose}")

            try:
                response = int(input(">> "))
                selected_pose = enumerated_poses[response][1]

                goal_msg = StaticPose.Goal()
                goal_msg.pose = selected_pose

                self.get_logger().info('Sending goal request...')
                print(self.pose_action_client.send_goal_async(goal_msg))

            except Exception as e:
                print(e)


    def sequenceMenu(self):
        self.sequence_action_client = ActionClient(
            self,
            PoseSequence,
            '/pose_sequence')
        
        self.get_logger().info('Waiting for sequence server...')
        self.sequence_action_client.wait_for_server()

        enumerated_sequences = list(enumerate(list(sequences.keys())))

        while True:
            for (index, pose) in enumerated_sequences:
                print(f"{index}. {pose}")

            try:
                response = int(input(">> "))
                selected_pose = enumerated_sequences[response][1]

                goal_msg = StaticPose.Goal()
                goal_msg.pose = selected_pose

                self.get_logger().info('Sending goal request...')
                print(self.sequence_action_client.send_goal_async(goal_msg))

            except Exception as e:
                print(e)


def main(args=None):
    rclpy.init(args=args)

    action_menu = SenceActionMenu()

    rclpy.spin(action_menu)

    action_menu.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()