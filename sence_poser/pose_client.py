import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from sence_msgs.action import StaticPose

from .sence_poses import poses


class PoseClientMenu(Node):
    def __init__(self):
        super().__init__('pose_client_menu')

        self.pose_action_client = ActionClient(
            self,
            StaticPose,
            '/static_pose')

        self.get_logger().info('Waiting for action server...')
        self.pose_action_client.wait_for_server()

        print("Sence Pose Menu")

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
                self.pose_action_client.send_goal_async(goal_msg)

            except Exception as e:
                print(e)
        return


def main(args=None):
    rclpy.init(args=args)

    pose_client_menu = PoseClientMenu()

    rclpy.spin(pose_client_menu)

    pose_client_menu.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()