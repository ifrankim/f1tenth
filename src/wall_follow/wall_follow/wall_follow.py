import math
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import time


class WallFollowNode(Node):

    def __init__(self):
        super().__init__("wall_follow")

        self.velocity_x = 0.0
        self.pid_i = 0.0
        self.previous_error = 0.0
        self.last_callback_time = time.time()
        self.prev_steering_angle = 0

        self.declare_parameters(
            namespace="",
            parameters=[
                ("look_ahead_distance", 12.5),
                ("kp", 1.0),
                ("ki", 0.0),
                ("kd", 0.09),
                ("desired_distance", 0.998),
                ("a_angle", 10),
                ("b_angle", 90),
            ],
        )

        self.subscription = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )
        self.subscription = self.create_subscription(
            Odometry, "ego_racecar/odom", self.odom_callback, 10
        )
        self.publisher = self.create_publisher(AckermannDriveStamped, "drive", 10)

    def odom_callback(self, msg):
        self.velocity_x = msg.twist.twist.linear.x

    def scan_callback(self, msg):
        if self.last_callback_time is not None:
            steering_angle = self.__steering_angle_control__(msg)
            speed = self.__speed_control__(steering_angle)

            self.prev_steering_angle = steering_angle

            # self.get_logger().info(f"angle: {steering_angle*180/math.pi:.3f}")

            drive_msg = AckermannDriveStamped()
            drive_msg.drive.steering_angle = steering_angle
            drive_msg.drive.speed = speed
            self.publisher.publish(drive_msg)

    def __speed_control__(self, steering_angle):
        speed = 0.0
        if abs(steering_angle) >= self.__to_radians__(0) and abs(
            steering_angle
        ) < self.__to_radians__(10):
            speed = 1.5
        elif abs(steering_angle) >= self.__to_radians__(10) and abs(
            steering_angle
        ) < self.__to_radians__(20):
            speed = 1.0
        else:
            speed = 0.5
        return speed

    def __steering_angle_control__(self, scan_data):
        current_time = time.time()
        dt = current_time - self.last_callback_time
        error = self.__get_error__(scan_data)

        steering_angle = self.__pid_control__(error, dt)

        self.get_logger().info(
            f"error:{error:.3f} angle: {steering_angle*180/math.pi:.3f}"
        )

        self.previous_error = error
        self.last_callback_time = current_time

        return steering_angle

    def __get_range_index__(self, scan_data, angle):
        return int(
            len(scan_data.ranges)
            - (
                (scan_data.angle_max - self.__to_radians__(angle))
                / scan_data.angle_increment
            )
        )

    def __get_error__(self, scan_data):
        ranges = scan_data.ranges
        angle_increment = scan_data.angle_increment

        l = self.get_parameter("look_ahead_distance").value  # self.velocity_x * 0.2

        a_range = self.__get_range_index__(
            scan_data, self.get_parameter("a_angle").value
        )
        b_range = self.__get_range_index__(
            scan_data, self.get_parameter("a_angle").value
        )

        a_distance = ranges[a_range]
        b_distance = ranges[b_range]

        theta = angle_increment * abs(a_range - b_range)

        alpha = math.atan2(
            (a_distance * math.cos(theta) - b_distance), a_distance * math.sin(theta)
        )

        dt_distance = b_distance * math.cos(alpha)

        # middle_index = len(ranges) // 2
        # dt_distance = min(ranges[middle_index:])
        # alpha = math.acos(dt_distance/b_distance)

        desired_distance = self.get_parameter("desired_distance").value
        # self.get_logger().info(f"d:{dt_distance:.3f}")

        dt_distance_1 = dt_distance + l * math.sin(alpha)
        error = desired_distance - dt_distance_1
        # if (dt_distance > desired_distance): error *= -1
        self.get_logger().info(
            f"e:{error:.3f}, dt:{dt_distance:.3f} desr:{desired_distance:.3f}"
        )

        return error

    def __pid_control__(self, error, dt):
        kp = self.get_parameter("kp").value
        ki = self.get_parameter("ki").value
        kd = self.get_parameter("kd").value

        pid_p = kp * error
        pid_d = kd * (error - self.previous_error)
        self.pid_i = (self.pid_i + error) * dt * ki

        steering_angle = pid_p + self.pid_i + pid_d

        if steering_angle > self.__to_radians__(90):
            steering_angle = self.__to_radians__(90)
        if steering_angle < -self.__to_radians__(90):
            steering_angle = -self.__to_radians__(90)

        return -steering_angle

    def __to_radians__(self, angle):
        return math.pi * angle / 180.0


def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollowNode()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
