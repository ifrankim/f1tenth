import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry


class SimulatedImuNode(Node):
    def __init__(self):
        super().__init__("simulated_imu")
        self.imu_publisher = self.create_publisher(Imu, "imu_topic", 10)
        self.odom_subscription = self.create_subscription(
            Odometry,
            'ego_racecar/odom',
            self.odom_callback,
            10)

    def odom_callback(self, msg):
        imu_msg = Imu()
        imu_msg.header = msg.header
        
        imu_msg.frame_id = "ego_racecar/base_link"
        
        imu_msg.linear_acceleration.x = msg.twist.twist.linear.x
        imu_msg.linear_acceleration.y = msg.twist.twist.linear.y
        imu_msg.linear_acceleration.z = msg.twist.twist.linear.z

        imu_msg.angular_velocity.x = msg.twist.twist.angular.x
        imu_msg.angular_velocity.y = msg.twist.twist.angular.y
        imu_msg.angular_velocity.z = msg.twist.twist.angular.z

        imu_msg.linear_acceleration.x += self.add_noise()
        imu_msg.linear_acceleration.y += self.add_noise()
        imu_msg.linear_acceleration.z += self.add_noise()

        imu_msg.angular_velocity.x += self.add_noise()
        imu_msg.angular_velocity.y += self.add_noise()
        imu_msg.angular_velocity.z += self.add_noise()

        self.imu_publisher.publish(imu_msg)
        
    def add_noise(self):
        return random.uniform(-0.1, 0.1)

def main(args=None):
    rclpy.init(args=args)
    simulated_imu_node = SimulatedImuNode()
    rclpy.spin(simulated_imu_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
