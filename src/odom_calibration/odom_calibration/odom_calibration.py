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
        self.csv_writer = csv.writer(
            self.csv_file, delimiter=",", quotechar='"', quoting=csv.QUOTE_MINIMAL
        )
        self.csv_writer.writerow(["x", "y", "z"])

    def odom_callback(self, msg):
        position = msg.pose.pose.position
        self.csv_writer.writerow([position.x, position.y, position.z])


def main(args=None):
    rclpy.init(args=args)

    odom_subscriber = OdomCalibrationNode()

    rclpy.spin(odom_subscriber)

    odom_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
