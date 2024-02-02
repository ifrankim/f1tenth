
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
        self.pid_i = 0.0
        self.previous_error = 0.0

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
        if steering_angle >= 0 and steering_angle < 0.18:
            speed = 1.5 
        elif steering_angle >= 0.18 and steering_angle < 0.35:
            speed = 1.0
        else:
            speed = 0.5
        return speed

    def __steering_angle_control__(self, scan_data):
        ranges = scan_data.ranges
        angle_increment = scan_data.angle_increment
        time = 2
        l = self.velocity_x*time

        a_range = len(ranges) - 419
        b_range = len(ranges) - 177
        a_distance = ranges[a_range]
        b_distance = ranges[b_range]
        # dt_distance = min(ranges)
        self.get_logger().info(f"a_distance:{a_distance} a_range: {a_range}")

        theta = angle_increment * abs(a_range - b_range)
        
        alpha = (math.atan((a_distance*math.cos(theta) - b_distance)/a_distance*math.sin(theta)))

        dt_distance = b_distance*math.cos(alpha)
        dt_distance_1 = dt_distance + l*math.sin(alpha)

        desired_distance = 0.4
        error = desired_distance - dt_distance_1

        #PID
        kp = 0.3
        ki = 0
        kd = 0

        pid_p = kp*error
        pid_d = kd * (error - self.previous_error)/time
        
        if (-3 < error and error < 3):
            self.pid_i = self.pid_i + (ki * error)
        else:
            self.pid_i = 0

        steering_angle = pid_p + self.pid_i + pid_d

        self.previous_error = error

        # self.get_logger().info(f"a_distance:{a_distance} error: {error}, angle: {steering_angle}")

        return steering_angle

def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollowNode()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()