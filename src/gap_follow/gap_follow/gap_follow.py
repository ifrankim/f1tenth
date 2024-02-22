from collections import deque
import rclpy
from rclpy.node import Node

import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive

import matplotlib.pyplot as plt
from threading import Thread
import queue


class PlotThread(Thread):
    def __init__(self, data_queue):
        super().__init__()
        self.data_queue = data_queue
        self.running = True

    def run(self):
        while self.running:
            try:
                data = self.data_queue.get(timeout=1)
                self.update_plot(data)
            except queue.Empty:
                pass

    def update_plot(self, data):
        proc_ranges, max_gap_start, max_gap_end, central_index, virtual_lidar = data

        plt.clf()
        plt.plot(range(len(proc_ranges)), proc_ranges, label="lidar proc")
        plt.axvline(x=max_gap_start, color="r", label="max_gap_start")
        plt.axvline(x=max_gap_end, color="r", label="max_gap_end")
        plt.axvline(x=central_index, color="b", label=f"index to follow")
        plt.plot(range(len(virtual_lidar)), virtual_lidar, label="virtual_lidar")
        plt.legend()
        plt.pause(0.001)


class ReactiveFollowGap(Node):
    """
    Implement Wall Following on the car
    This is just a template, you are free to implement your own node!
    """

    def __init__(self):
        super().__init__("gap_follow")
        self.subscription = self.create_subscription(
            LaserScan, "scan", self.scan_callback, 10
        )
        self.publisher = self.create_publisher(AckermannDriveStamped, "drive", 10)

        self.fov_angle = 90

        self.data_queue = queue.Queue()
        self.plot_thread = PlotThread(self.data_queue)
        self.plot_thread.start()

    def preprocess_lidar(self, scan_data):
        """Preprocess the LiDAR scan array.
        1.Restricting the FOV to -50deg to +50deg
        1.Setting each value to the mean over some window
        2.Rejecting high values (eg. > 5m)
        """
        # plt.plot(range(len(scan_data.ranges)), (scan_data.ranges), label="lidar raw")

        initial_fov = int(
            self.__to_radians__(134 - self.fov_angle) / scan_data.angle_increment
        )
        end_fov = int(
            self.__to_radians__(134 + self.fov_angle) / scan_data.angle_increment
        )
        proc_ranges = scan_data.ranges[initial_fov:end_fov]

        # all_zeros = np.ones_like(scan_data.ranges)
        # all_zeros[:initial_fov] = 0
        # all_zeros[end_fov:] = 0
        # proc_ranges = np.array(scan_data.ranges) * all_zeros

        temp = np.zeros(len(proc_ranges))
        running_avg = deque()
        window_length = 7
        for i, d in enumerate(proc_ranges):
            running_avg.append(d)
            if len(running_avg) > window_length:
                running_avg.popleft()
            temp[i] = np.mean(running_avg)

        proc_ranges = np.clip(proc_ranges, 0, 5)

        # plt.plot(range(len(proc_ranges)), (proc_ranges), label="lidar proc")
        # plt.legend()
        # plt.show()

        return proc_ranges.tolist()  # temp.tolist()

    def find_max_gap(self, ranges):
        """Return the start index & end index of the max gap in ranges"""
        t = 2
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

        self.get_logger().info(
            f"max_gap_start: {max_gap_start} max_gap_end:{max_gap_end}"
        )
        # plt.axvline(x=max_gap_start, color='r', label='max_gap_start')
        # plt.axvline(x=max_gap_end, color='r', label='max_gap_end')
        # plt.legend()
        # plt.show()

        return max_gap_start, max_gap_end

    def find_best_point(
        self, start_i, end_i, processed_ranges, angle_increment, ranges
    ):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return angle for the best point in processed_ranges
        """

        max_value = max(processed_ranges[start_i:end_i])
        max_value_indices = [
            i
            for i, val in enumerate(processed_ranges[start_i:end_i], start=start_i)
            if val == max_value
        ]

        central_index = max_value_indices[len(max_value_indices) // 2]
        # index = processed_ranges.index(max(processed_ranges[start_i:end_i]))

        # plt.axvline(x=central_index, color='b', label=f"index to follow")
        # plt.legend()
        central_index = central_index + int(
            self.__to_radians__(134 - self.fov_angle) / angle_increment
        )

        angle = -self.__to_radians__(134) + angle_increment * central_index
        self.get_logger().info(f"angle: {angle*180/math.pi:.3f} ndex:{central_index}")

        return (
            angle,
            central_index
            - int(self.__to_radians__(134 - self.fov_angle) / angle_increment),
            ranges[int(self.__to_radians__(134))],
        )

    def find_disparities(self, ranges, angle_increment):
        """Find disparities in lidar readings and extend them to create a virtual lidar."""
        car_width = 0.6
        threshold = 0.25

        virtual_lidar = ranges[:]

        for i in range(1, len(ranges) - 1):
            current_range = ranges[i]
            prev_range = ranges[i - 1]

            if abs(current_range - prev_range) > threshold:
                # disparity_angle = math.acos(1 - (car_width**2) / (2 * max(current_range, prev_range)**2))

                disparity_angle = math.atan2(car_width, current_range)
                disparity_angle_index = int(disparity_angle / angle_increment)

                extend_direction = 1 if prev_range < current_range else -1

                for j in range(disparity_angle_index + 1):
                    extend_index = i + j * extend_direction
                    if 0 <= extend_index < len(virtual_lidar):
                        virtual_lidar[extend_index] = min(prev_range, current_range)
                        self.get_logger().info(
                            f"i:{i} cur:{current_range:.2f} prev:{prev_range:.2f} angle:{disparity_angle*180/math.pi:.3f} idic:{extend_direction} exti:{extend_index} range:{min(prev_range, current_range)}"
                        )

        # plt.plot(range(len(virtual_lidar)), (virtual_lidar), label="virtual_lidar")
        # plt.legend()
        # plt.show()
        return virtual_lidar

    def __speed_control__(self, forward_distance):
        """Controls the velocity based on the distance in front of the car"""
        min_safe_distance = 1.0
        full_speed_distance = 2.0
        max_speed = 1.5
        min_speed = 0.0

        if forward_distance >= full_speed_distance:
            speed = max_speed
        elif forward_distance <= min_safe_distance:
            speed = min_speed
        else:
            speed = min_speed + (max_speed - min_speed) * ((forward_distance - min_safe_distance) / (full_speed_distance - min_safe_distance))
        
        return speed


    def scan_callback(self, scan_data):
        """Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message"""
        self.get_logger().info(
            f"\n============================================================="
        )
        ranges = self.preprocess_lidar(scan_data)
        proc_ranges = ranges[:]
        ranges = self.find_disparities(ranges, scan_data.angle_increment)
        virtual_lidar = ranges[:]
        start_i, end_i = self.find_max_gap(ranges)
        steering_angle, central_index, distance = self.find_best_point(
            start_i, end_i, ranges, scan_data.angle_increment, scan_data.ranges
        )
        # plt.show()
        self.data_queue.put((proc_ranges, start_i, end_i, central_index, virtual_lidar))

        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        self.get_logger().info(f"angle: {steering_angle*180/math.pi:.3f}")
        speed = self.__speed_control__(distance)
        drive_msg.drive.speed = speed * 0.3
        self.publisher.publish(drive_msg)

    def __to_radians__(self, angle):
        return math.pi * angle / 180.0


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
