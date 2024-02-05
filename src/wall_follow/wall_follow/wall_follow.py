import math
import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import matplotlib.pyplot as plt
import time


class WallFollowNode(Node):

    def __init__(self):
        super().__init__("wall_follow")

        self.velocity_x = 0.0
        self.pid_i = 0.0
        self.previous_error = 0.0
        self.last_callback_time = 0

        self.subscription = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )
        self.subscription = self.create_subscription(
            Odometry, "ego_racecar/odom", self.odom_callback, 10
        )
        self.publisher = self.create_publisher(AckermannDriveStamped, "drive", 10)

        self.points = []

    def scan_callback(self, msg):
        if self.last_callback_time is not None:
            if len(self.points) < 3000:
                steering_angle = self.__steering_angle_control__(msg)
                speed = self.__speed_control__(steering_angle)

                drive_msg = AckermannDriveStamped()
                drive_msg.drive.steering_angle = steering_angle
                drive_msg.drive.speed = speed
                self.publisher.publish(drive_msg)
            else:
                plt.plot(list(range(0, len(self.points), 1)), self.points)
                plt.axhline(y=0.5, color="r", linestyle="--", label="Linha reta em y=5")
                plt.show()

    def odom_callback(self, msg):
        self.velocity_x = msg.twist.twist.linear.x

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
        # print("Interval between callbacks:", interval)
        error = self.__get_error__(scan_data)

        steering_angle = self.__pid_control__(error, dt)
        self.previous_error = error
        self.last_callback_time = current_time

        self.get_logger().info(
            f"error: {error:.3f}, angle: {steering_angle*180/math.pi:.3f}"
        )
        return steering_angle

    def __get_error__(self, scan_data):
        ranges = scan_data.ranges
        angle_increment = scan_data.angle_increment

        l = self.velocity_x * 2

        a_range = 419
        b_range = 177

        # a_range = len(ranges) - 419
        # b_range = len(ranges) - 177

        a_distance = ranges[a_range]
        b_distance = ranges[b_range]

        theta = angle_increment * abs(a_range - b_range)

        alpha = math.atan(
            (a_distance * math.cos(theta) - b_distance) / a_distance * math.sin(theta)
        )

        dt_distance = b_distance * math.cos(alpha)
        dt_distance_1 = dt_distance + l * math.sin(alpha)

        desired_distance = 0.5
        error = desired_distance - dt_distance_1

        self.points.append(dt_distance_1)

        return error

    def __pid_control__(self, error, dt):
        kp = 3
        ki = 0
        kd = 0.1

        pid_p = kp * error
        pid_d = kd * (error - self.previous_error) / dt

        if -3 < error and error < 3:
            self.pid_i = (self.pid_i + error) * dt * ki
        else:
            self.pid_i = 0

        steering_angle = pid_p + self.pid_i + pid_d

        if steering_angle > 80 * math.pi / 180:
            steering_angle = 80 * math.pi / 180
        if steering_angle < -80 * math.pi / 180:
            steering_angle = -80 * math.pi / 180

        # self.get_logger().info(f"p:{pid_p:.3f} i: {self.pid_i:.3f}, d: {pid_d:.3f} angle: {steering_angle:.3f}")

        return steering_angle

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
