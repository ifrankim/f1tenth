import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class TfTransformNode(Node):
    def __init__(self):
        super().__init__("tf_transform")

        # self.static_tf_publisher = self.create_publisher(
        #     TransformStamped, "/tf_static", 1
        # )
        self.tf_broadcasater = TransformBroadcaster(self)

        self.timer = self.create_timer(0.003, self.timer_callback)

    def timer_callback(self):
        static_transform_stamped = TransformStamped()
        static_transform_stamped.header.frame_id = "global_map"
        static_transform_stamped.child_frame_id = "map"
        static_transform_stamped.transform.translation.x = 0.0
        static_transform_stamped.transform.translation.y = 0.0
        static_transform_stamped.transform.translation.z = 0.0
        static_transform_stamped.transform.rotation.w = 1.0
        static_transform_stamped.header.stamp = self.get_clock().now().to_msg()
        # self.static_tf_publisher.publish(static_transform_stamped)
        self.tf_broadcasater.sendTransform(static_transform_stamped)


def main(args=None):
    rclpy.init(args=args)
    static_tf_publisher = TfTransformNode()
    rclpy.spin(static_tf_publisher)
    static_tf_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
