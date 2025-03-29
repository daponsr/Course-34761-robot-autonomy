#!/usr/bin/env python3

import os
import sys
import time
import rclpy
import math

from rclpy.node import Node
from sensor_msgs.msg import LaserScan

# node to subscribe to the lidar data

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        # print the range of the lidar data
        ranges = msg.ranges
        print('Ranges: ', len(ranges))
        min_distance_to_obstacle = min(ranges)
        print('Minimum distance to obstacle: ', min_distance_to_obstacle)
        max_distance_to_obstacle = max(ranges)
        print('Maximum distance to obstacle: ', max_distance_to_obstacle)

def main(args=None):
    rclpy.init(args=args)

    lidar_subscriber = LidarSubscriber()

    rclpy.spin(lidar_subscriber)

    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
