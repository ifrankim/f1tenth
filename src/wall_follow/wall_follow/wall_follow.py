
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
        collision_imminent = self.check_tts(msg)

        if collision_imminent:
            drive_msg = AckermannDriveStamped()
            drive_msg.drive.speed = 0.0 
            self.publisher.publish(drive_msg)

    def odom_callback(self, msg):
        self.velocity_x = msg.twist.twist.linear.x

    def check_tts(self, scan_data):
        ranges = scan_data.ranges
        angle_max = 2.3499999046325684
        angle_min = -angle_max

        a_range = 419
        b_range = 177
        a_distance = range[a_range]
        b_distance = range[b_range]
        # dt_distance = min(ranges)
        theta = (abs(angle_max)+abs(angle_min))/len(ranges) * (a_range - b_range)

        
        alpha = 1/(math.tan((a_distance*math.cos(theta) - b_distance)/alpha*math.sin(theta)))

        dt_distance = b_distance*math.cos(alpha)

        for i in range(len(ranges)):
            current_range = ranges[i]
            
            theta = (abs(angle_max)+abs(angle_min))/len(ranges) * i

            if math.isnan(current_range) or self.velocity_x*math.cos(theta-angle_max) == 0:
                continue
            
            ttc = current_range/((self.velocity_x)*math.cos(theta-angle_max))
            if (ttc < 0.9 and ttc > 0):
                self.get_logger().info(f"TTC: {ttc}")
                return True

        return False

def main(args=None):
    rclpy.init(args=args)
    wall_follow_node = WallFollowNode()
    rclpy.spin(wall_follow_node)
    wall_follow_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()