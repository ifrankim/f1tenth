import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
import csv
import math


class OdomCalibrationNode(Node):

    def __init__(self):
        super().__init__("odom_subscriber")
        self.subscription = self.create_subscription(
            Odometry, "ego_racecar/odom", self.odom_callback, 10
        )
        self.csv_file = open("trajectory.csv", mode="w")
        # self.csv_writer = csv.writer(
        #     self.csv_file, delimiter=",", quotechar='"', quoting=csv.QUOTE_MINIMAL
        # )
        # self.csv_writer.writerow(["x", "y", "z"])
        self.publisher = self.create_publisher(AckermannDriveStamped, "drive", 10)

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.distance_traveled = position.x

    def move_straight(self):
        speed = 0.0
        if self.distance_traveled < 4.5:
            speed = 1.2
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = 0.0
        drive_msg.drive.speed = speed
        self.publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)

    odom_subscriber = OdomCalibrationNode()

    rclpy.spin(odom_subscriber)

    odom_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
