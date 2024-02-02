
import math
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class WallFollowNode(Node):

    def __init__(self):
        super().__init__('wall_follow')
        
        self.velocity_x = 0.0

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            10)
        self.subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.odom_callback,
            10)
        self.publisher = self.create_publisher(AckermannDriveStamped, 'drive', 10)

    def scan_callback(self, msg):
        steering_angle = self.__steering_angle_control__(msg)
        speed = self.__speed_control__(steering_angle)

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.publisher.publish(drive_msg)

    def odom_callback(self, msg):
        self.velocity_x = msg.twist.twist.linear.x

    def __speed_control__(self, steering_angle):
        speed = 0.0
        if steering_angle >= 0 and steering_angle < 10:
            speed = 1.5
        elif steering_angle >= 10 and steering_angle < 20:
            speed = 1.0
        else:
            speed = 0.5
        return speed

    def __steering_angle_control__(self, scan_data):
        ranges = scan_data.ranges
        angle_max = 2.3499999046325684
        angle_min = -angle_max
        l = self.velocity_x*0.25

        a_range = 419
        b_range = 177
        a_distance = range[a_range]
        b_distance = range[b_range]
        # dt_distance = min(ranges)
        theta = (abs(angle_max)+abs(angle_min))/len(ranges) * (a_range - b_range)
        
        alpha = 1/(math.tan((a_distance*math.cos(theta) - b_distance)/a_distance*math.sin(theta)))

        dt_distance = b_distance*math.cos(alpha)

        dt_distance_1 = dt_distance + l*math.sin(alpha)


def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollowNode()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()