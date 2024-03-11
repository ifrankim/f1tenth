import random
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class NoisyOdomNode(Node):
    def __init__(self):
        super().__init__("noisy_odom")
        self.odom_publisher = self.create_publisher(Odometry, "noisy_odom", 10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.odom_callback,
            10)

    def odom_callback(self, msg):
        odom_msg = Odometry()

        odom_msg.header = msg.header
        
        odom_msg.child_frame_id = msg.child_frame_id

        odom_msg.pose.pose = msg.pose.pose

        odom_msg.twist.twist = msg.twist.twist

        odom_msg.pose.pose.position.x += self.add_noise()
        odom_msg.pose.pose.position.y += self.add_noise()
        odom_msg.pose.pose.position.z += self.add_noise()
        
        odom_msg.twist.twist.linear.x += self.add_noise()
        odom_msg.twist.twist.linear.y += self.add_noise()
        odom_msg.twist.twist.linear.z += self.add_noise()

        odom_msg.twist.twist.angular.x += self.add_noise()
        odom_msg.twist.twist.angular.y += self.add_noise()
        odom_msg.twist.twist.angular.z += self.add_noise()

        self.odom_publisher.publish(odom_msg)
        
    def add_noise(self):
        return random.uniform(-0.1, 0.1)

def main(args=None):
    rclpy.init(args=args)
    noisy_odom_node = NoisyOdomNode()
    rclpy.spin(noisy_odom_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
