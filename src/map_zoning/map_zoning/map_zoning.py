import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import cv2


class MapZoningNode(Node):

    def __init__(self):
        super().__init__("map_zoning")

        self.declare_parameters(
            namespace="",
            parameters=[
                ("image_path", "/home/rilbasestation02/sim_ws/src/f1tenth_gym_ros/maps/icra_2023_mod_color.jpeg"),
                ("car_origin", (10.049340820312501, 4.060702514648438)),
                ("pixel_scale", 0.05),
            ],
        )

        self.subscription = self.create_subscription(
            Odometry, "ego_racecar/odom", self.odom_callback, 10
        )

    def odom_callback(self, msg):
        velocity_x = msg.pose.pose.position
        color = self.get_color_at_position((velocity_x.x, velocity_x.y))
        self.get_logger().info(f"{color}")

    def convert_position_to_pixel(self, image, car_position):
        pixel_scale = self.get_parameter("pixel_scale").value  # 1 px : 0.05 m
        car_origin = self.get_parameter("car_origin").value  # canto inferior esquerdo
        car_origin = (
            car_origin[0],
            len(image) * pixel_scale - car_origin[1],
        )  # canto inferior direito (mesmo da imagem)

        # Converter coordenadas do carro para coordenadas do mapa
        map_position = (
            car_origin[0] + car_position[0],
            car_origin[1] - car_position[1],
        )

        # Converter coordenadas do mapa para pixels na imagem
        pixel_position = (
            int(map_position[0] / pixel_scale),
            int(map_position[1] / pixel_scale),
        )
        return pixel_position

    def get_color_at_position(self, position):
        img_path = self.get_parameter("image_path").value
        map_image = cv2.imread(img_path)
        x, y = self.convert_position_to_pixel(map_image, position)
        color = map_image[y, x]  # BGR, então a ordem dos canais é invertida

        if color[0] > color[1] and color[0] > color[2]:
            color = "blue"
        elif color[1] > color[0] and color[1] > color[2]:
            color = "green"
        else:
            color = "red"
        return color


def main(args=None):
    rclpy.init(args=args)
    map_zoning_node = MapZoningNode()
    rclpy.spin(map_zoning_node)
    map_zoning_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
