from collections import deque
import rclpy
from rclpy.node import Node

import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped
from example_interfaces.srv import SetBool

class DisparityExtender(Node):
    """
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """

    def __init__(self):
        super().__init__("disparity_extender")

        self.declare_parameters(
            namespace="",
            parameters=[("max_speed", 1.0), ("car_width", 0.8), ("fov_angle", 70)],
        )

        self.subscription = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )
        self.publisher = self.create_publisher(AckermannDriveStamped, "multiplexer/drive", 10)

        self.service = self.create_service(
            SetBool, "safety_service", self.safety_service_callback
        )

        self.fov_angle = self.get_parameter("fov_angle").value
        self.lidar_data = {}

    def safety_service_callback(self, request, response):
        """Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message"""
        # self.get_logger().info(
        #     f"\n============================================================="
        # )
        
        ranges = self.preprocess_lidar(self.lidar_data)
        ranges = self.find_disparities(ranges, self.lidar_data.angle_increment)
        start_i, end_i = self.find_max_gap(ranges)
        steering_angle, distance = self.find_best_point(
            start_i, end_i, ranges, self.lidar_data.angle_increment
        )
        speed = self.__speed_control__(distance)
        obs_detected = self.__handle_rear__(self.lidar_data)
        # if obs_detected != 0:
        #     # speed = 0.5
        #     steering_angle = (
        #         self.__to_radians__(10)
        #         if obs_detected > 0
        #         else self.__to_radians__(-10)
        #     )

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        drive_msg.drive.speed = speed
        self.publisher.publish(drive_msg)

        response.success = True
        return response

    def preprocess_lidar(self, scan_data):
        """Preprocess the LiDAR scan array.
        1.Restricting the FOV to -fov_angle deg to +fov_angle deg
        1.Setting each value to the mean over some window
        2.Rejecting high values (eg. > 5m)
        """
        self.fov_angle = self.get_parameter("fov_angle").value
        initial_fov = int(
            self.__to_radians__(134 - self.fov_angle) / scan_data.angle_increment
        )
        end_fov = int(
            self.__to_radians__(134 + self.fov_angle) / scan_data.angle_increment
        )
        proc_ranges = scan_data.ranges[initial_fov:end_fov]

        temp = np.zeros(len(proc_ranges))
        running_avg = deque()
        window_length = 7
        for i, d in enumerate(proc_ranges):
            running_avg.append(d)
            if len(running_avg) > window_length:
                running_avg.popleft()
            temp[i] = np.mean(running_avg)

        proc_ranges = np.clip(temp, 0, 5)

        return proc_ranges.tolist()

    def find_max_gap(self, ranges):
        """Return the start index & end index of the max gap in ranges"""
        t = 5
        max_gap_length = 0
        max_gap_start = 0
        max_gap_end = 0
        current_start = 0
        current_length = 0

        while max_gap_start == max_gap_end:
            for i in range(len(ranges)):
                if ranges[i] > t:
                    current_length += 1
                    if current_length > max_gap_length:
                        max_gap_length = current_length
                        max_gap_start = current_start
                        max_gap_end = i
                else:
                    current_start = i + 1
                    current_length = 0
            t -= 0.1
            if max_gap_start == max_gap_end or max_gap_start > max_gap_end:
                max_gap_length = 0
                max_gap_start = 0
                max_gap_end = 0
                current_start = 0
                current_length = 0

        return max_gap_start, max_gap_end

    def find_best_point(self, start_i, end_i, processed_ranges, angle_increment):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return angle for the best point in processed_ranges
        """
        self.fov_angle = self.get_parameter("fov_angle").value
        max_value = max(processed_ranges[start_i:end_i])
        max_value_indices = [
            i
            for i, val in enumerate(processed_ranges[start_i:end_i], start=start_i)
            if val == max_value
        ]

        central_index = max_value_indices[len(max_value_indices) // 2]
        central_index = central_index + int(
            self.__to_radians__(134 - self.fov_angle) / angle_increment
        )

        angle = -self.__to_radians__(134) + angle_increment * central_index

        return (
            angle,
            processed_ranges[len(processed_ranges) // 2],
        )

    def find_disparities(self, ranges, angle_increment):
        """Find disparities in lidar readings and extend them to create a virtual lidar."""
        car_width = self.get_parameter("car_width").value
        threshold = 0.2

        virtual_lidar = ranges[:]

        for i in range(1, len(ranges) - 1):
            current_range = ranges[i]
            prev_range = ranges[i - 1]

            if abs(current_range - prev_range) > threshold:
                disparity_distance = max(current_range, prev_range)
                disparity_angle = math.atan2(car_width, disparity_distance)
                disparity_angle_index = int(disparity_angle / angle_increment)

                extend_direction = 1 if prev_range < current_range else -1

                for j in range(disparity_angle_index + 1):
                    extend_index = i + j * extend_direction
                    if 0 <= extend_index < len(virtual_lidar):
                        virtual_lidar[extend_index] = min(
                            prev_range, current_range, virtual_lidar[extend_index]
                        )

        return virtual_lidar

    def __speed_control__(self, forward_distance):
        """Controls the velocity based on the distance in front of the car"""
        min_safe_distance = 0.2
        full_speed_distance = 2.0
        max_speed = self.max_speeds = self.get_parameter("max_speed").value
        min_speed = 0.0

        if forward_distance >= full_speed_distance:
            speed = max_speed
        elif forward_distance <= min_safe_distance:
            speed = min_speed
        else:
            speed = min_speed + (max_speed - min_speed) * (
                (forward_distance - min_safe_distance)
                / (full_speed_distance - min_safe_distance)
            )

        # self.get_logger().info(f"speed:{speed} dist:{forward_distance:.2f}")
        return speed

    def __handle_rear__(self, scan_data):
        """Handle rear obstacle detection based on LiDAR scan data.
        Detects obstacles behind the car to prevent collisions when turning.
        """
        self.fov_angle = self.get_parameter("fov_angle").value
        initial_fov = int(
            self.__to_radians__(134 - self.fov_angle - 10) / scan_data.angle_increment
        )
        end_fov = int(
            self.__to_radians__(134 + self.fov_angle + 10) / scan_data.angle_increment
        )
        right_fov = scan_data.ranges[:initial_fov]
        left_fov = scan_data.ranges[end_fov:]

        obs_detected = 0
        min_safe_distance = 0.2

        for current_range in right_fov:
            if current_range <= min_safe_distance:
                obs_detected = 1
                break
        if ~obs_detected:
            for current_range in left_fov:
                if current_range <= min_safe_distance:
                    obs_detected = -1
                    break

        return obs_detected

    def scan_callback(self, scan_data):
        """Saves the lidar data into a variable"""
        self.lidar_data = scan_data
        
    def __to_radians__(self, angle):
        return math.pi * angle / 180.0


def main(args=None):
    rclpy.init(args=args)
    print("Disparity Extender Initialized")
    reactive_node = DisparityExtender()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
