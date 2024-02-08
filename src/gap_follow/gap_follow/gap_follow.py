from collections import deque
import rclpy
from rclpy.node import Node

import math
import numpy as np
from sensor_msgs.msg import LaserScan
from ackermann_msgs.msg import AckermannDriveStamped, AckermannDrive


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

    def preprocess_lidar(self, scan_data):
        """Preprocess the LiDAR scan array.
        1.Setting each value to the mean over some window
        2.Rejecting high values (eg. > 3m)
        """
        proc_ranges = np.clip(
            scan_data.ranges, scan_data.range_min, scan_data.range_max
        )

        temp = np.zeros(len(proc_ranges))
        running_avg = deque()
        window_length = 5
        for d in proc_ranges:
            running_avg.append(d)
            if len(running_avg) > window_length:
                running_avg.popleft()

            temp[len(running_avg) - 1] = np.mean(running_avg)

        return temp.tolist()

    def find_max_gap(self, ranges):
        """Return the start index & end index of the max gap in ranges"""
        t = 3
        start_index = None
        end_index = None
        max_consecutive_count = 0
        current_consecutive_count = 0

        for i, value in enumerate(ranges):
            if value > t:
                if current_consecutive_count == 0:
                    start_index = i
                current_consecutive_count += 1
            else:
                if current_consecutive_count > max_consecutive_count:
                    max_consecutive_count = current_consecutive_count
                    end_index = i - 1
                current_consecutive_count = 0

        if current_consecutive_count > max_consecutive_count:
            max_consecutive_count = current_consecutive_count
            end_index = len(ranges) - 1

        return start_index, end_index

    def find_best_point(self, start_i, end_i, ranges, angle_increment):
        """Start_i & end_i are start and end indicies of max-gap range, respectively
        Return index of best point in ranges
            Naive: Choose the furthest point within ranges and go there
        """
        #naive
        index = ranges.index(max(ranges[start_i:end_i]))
        return angle_increment * index

    

    def find_disparities(self, ranges, angle_increment):
        """Find disparities in lidar readings and 
        for each disparity, extend it half the width of the car
        """
        car_width = 0.296 / 2

        for i, current_range in enumerate(ranges):
            if i == 0:
                continue

            last_range = ranges[i - 1]
            diff = last_range - current_range
            if abs(diff) > 3:
                theta = math.acos(1 - (car_width**2) / (2 * max(current_range, last_range)**2))
                extension = int(theta / angle_increment)
                for j in range(extension):
                    ranges[i - j] = current_range if diff > 0 else last_range
        return ranges

    def scan_callback(self, scan_data):
        """Process each LiDAR scan as per the Follow Gap algorithm & publish an AckermannDriveStamped Message"""
        ranges = self.preprocess_lidar(scan_data)
        ranges = self.find_disparities(ranges, scan_data.angle_increment)
        start_i, end_i = self.find_max_gap(ranges)
        steering_angle = self.find_best_point(start_i, end_i, ranges, scan_data.angle_increment)
        
        drive_msg = AckermannDriveStamped()
        drive_msg.drive.steering_angle = steering_angle
        self.get_logger().info(f"angle: {steering_angle*180/math.pi:.3f}")
        
        drive_msg.drive.speed = 1.0
        self.publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    print("WallFollow Initialized")
    reactive_node = ReactiveFollowGap()
    rclpy.spin(reactive_node)

    reactive_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
