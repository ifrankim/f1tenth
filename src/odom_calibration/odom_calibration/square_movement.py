import rclpy
from rclpy.node import Node
from ackermann_msgs.msg import AckermannDriveStamped
import math


class SquareMovement(Node):
    def __init__(self):
        super().__init__("square_movement")
        self.publisher = self.create_publisher(AckermannDriveStamped, "drive", 10)
        self.timer = self.create_timer(5.0, self.move_square)
        self.counter = 0

    def move_square(self):
        if self.counter < 4:
            msg = AckermannDriveStamped()
            msg.drive.speed = 1.0
            msg.drive.acceleration = 0.5
            msg.drive.jerk = 0.1

            if self.counter % 2 == 0:
                msg.drive.steering_angle = 0.0
            else:
                msg.drive.steering_angle = math.pi / 2

            self.publisher.publish(msg)
            self.counter += 1
        else:
            msg = AckermannDriveStamped()
            msg.drive.speed = 0.0
            msg.drive.steering_angle = 0.0
            self.publisher.publish(msg)
            self.get_logger().info("Finished square movement")


def main(args=None):
    rclpy.init(args=args)

    square_movement = SquareMovement()

    rclpy.spin(square_movement)

    square_movement.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
