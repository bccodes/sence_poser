import rclpy
from rclpy.node import Node
from math import pi
from time import sleep

from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

jointNames = ['fl_joint1', 'fl_joint2', 'fl_joint3',
              'fr_joint1', 'fr_joint2', 'fr_joint3',
              'bl_joint1', 'bl_joint2', 'bl_joint3',
              'br_joint1', 'br_joint2', 'br_joint3']
flatPose = [0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0,
            0.0, 0.0, 0.0]
curlPose = [pi/2, pi/2, 2.0,
            -pi/2, -pi/2, 2.0,
            -pi/2, -pi/2, 2.0,
            pi/2, pi/2, pi/2]
crabStandPose = [-pi/2, pi/8, pi/6,
                 pi/2, -pi/8, pi/6,
                 pi/2, -pi/8, pi/6,
                 -pi/2, pi/8, pi/6]
dogReadyPose = [pi/2, 0.0, 0.0,
               -pi/2, 0.0, 0.0,
               -pi/2, 0.0, 0.0,
               pi/2, 0.0, 0.0]
dogFlatPose = [1.4, -pi/2, 0.2,
               -1.4, pi/2, 0.2,
               -1.8, -1.3, 0.2,
               1.8, 1.3, 0.2]
dogStandPose = [-0.9, -pi/2, 1.9,
               0.9, pi/2, 1.9,
               -4.0, -pi/2, 1.9,
               4.0, pi/2, 1.9]

class TrajectoryPublisher(Node):
    def __init__(self):
        super().__init__('trajectory_publisher')
        self.publisher_ = self.create_publisher(JointTrajectory, 'joint_trajectory_controller/joint_trajectory', 10)
        self.pose = flatPose
        self.poseTime = 1

    def poseMenu(self):
        while (True):
            try:
                print("""sence pose publisher
                time to pose = {ttp} seconds
                1: flat
                2: curl
                3: crab stand
                4: dog ready
                5: dog flat
                6: dog stand
                9: set time to pose
                0: quit""".format(ttp=self.poseTime)
                )
                response = int(input("enter a number: "))
                match response:
                    case 1:
                        self.pose = flatPose
                        self.publishPose()
                    case 2:
                        self.pose = curlPose
                        self.publishPose()
                    case 3:
                        self.pose = crabStandPose
                        self.publishPose()
                    case 4:
                        self.pose = dogReadyPose
                        self.publishPose()
                    case 5:
                        self.pose = dogFlatPose
                        self.publishPose()
                    case 6:
                        self.pose = dogStandPose
                        self.publishPose()
                    case 9:
                        self.changeSpeed()
                    case 0:
                        break;
                    case _:
                        print("unknown response")

            except Exception as e:
                print(e)

    def publishPose(self):
        msg = JointTrajectory()
        msg.joint_names = jointNames
        point1 = JointTrajectoryPoint()
        point1.positions = self.pose
        point1.time_from_start.sec = self.poseTime
        msg.points.append(point1)

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing now')

    def changeSpeed(self):
        try:
            self.poseTime = int(input("enter value in seconds: "))
        except Exception as e:
            print(e)


def main(args=None):
    rclpy.init(args=args)

    trajectory_publisher = TrajectoryPublisher()
    #rclpy.spin(trajectory_publisher)

    trajectory_publisher.poseMenu()

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    trajectory_publisher.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
