#!/usr/bin/env python3
import rclpy
from enum import Enum
from typing import List

import numpy as np

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.parameter import Parameter

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import TransformStamped
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult

from tf2_ros.buffer import Buffer
from tf2_ros import LookupException, TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster


from typing import Tuple

from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion
class State(Enum):
    FREE = 1
    OCCUPIED = 2
    UNKOWN = 3





class MyOccupancyGrid(Node):
    def __init__(self):
        super().__init__('my_occupancy_grid')
        # initilize so we do not publish the map before the first scan
        self.scan_has_been_received = False
        # define the grid
        self.width = 10
        self.height = 10
        self.resolution = float(0.1)
        self.min_prob = float(0.1)
        self.max_prob = float(0.9)
        self.prob_free = float(0.1)
        self.prob_occupied = float(0.9)
        self.prob_priori = float(0.5)

        # initialize the grid
        N = int(self.width / self.resolution)
        M = int(self.height / self.resolution)
        shape = (N, M)
        self.grid = np.ones(shape)

        # subscription to the lidar data
        self.subscription = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.subscription

        # publisher for the map --> where we will publish the map (OccupancyGrid)
        self.publisher = self.create_publisher(OccupancyGrid, '/map_2', 10)
        self.publisher

        # transfor buffer
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = StaticTransformBroadcaster(self)

        # the static transform from "odom" to "map"
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self.tf_broadcaster.sendTransform(tf)


    def scan_callback(self, msg):
        # get the current time
        current_time = self.get_clock().now()
        # get the transform from "base_link" to "odom"
        try:
            print("transofrm",'odom', msg.header.frame_id)
            transform = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, msg.header.stamp)
        except Exception as e:
            self.get_logger().error('Could not transform between odom and %s: %s' % (msg.header.frame_id, str(e)))
            return

        _, _, yaw = self.euler_from_quaternion(transform.transform.rotation)


        # convert to polar the msg
        polar = self.laser_scan_to_polar(msg)

        # convert to cartesian
        xy = self.polar_to_cartesian(polar, transform.transform.translation.x, transform.transform.translation.y, yaw)
        xy[:, 0] = (xy[:, 0] + self.width // 2) / self.resolution
        xy[:, 1] = (xy[:, 1] + self.height // 2) / self.resolution
        xy = xy.astype(int)
        xo = int((transform.transform.translation.x + self.width // 2) / self.resolution)
        yo = int((transform.transform.translation.y + self.height // 2) / self.resolution)
        
        # here we use bresenham to update the grid
        for i in range(xy.shape[0]):
            x = xy[i, 0]
            y = xy[i, 1]
            points= self.breseham(xo, yo, x, y)
            for j in range(points.shape[0] - 1):
                x = points[j, 0]
                y = points[j, 1]
                self.update_grid(x, y, State.FREE)
            x = xy[i, 0]
            y = xy[i, 1]
            if polar[i, 0] < msg.range_max:
                self.update_grid(x=x, y=y, state=State.OCCUPIED)
                self.update_grid(x=x - 1, y=y, state=State.OCCUPIED)
                self.update_grid(x=x + 1, y=y, state=State.OCCUPIED)
                self.update_grid(x=x, y=y + 1, state=State.OCCUPIED)
                self.update_grid(x=x, y=y - 1, state=State.OCCUPIED)

        # publish the map
        self.publish_map(current_time)

    def euler_from_quaternion(self, quaternion: Quaternion):
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        w = quaternion.w

        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        sinp = 2 * (w * y - z * x)
        pitch = np.arcsin(sinp)

        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    

    def update_grid(self, x: int, y: int, state: State):
        if x >= 0 and x < self.grid.shape[0] and y >= 0 and y < self.grid.shape[1]:
            if state == State.FREE:
                self.grid[x, y] = self.prob_free
            elif state == State.OCCUPIED:
                self.grid[x, y] = self.prob_occupied
            elif state == State.UNKOWN:
                self.grid[x, y] = self.prob_priori

    def breseham(self, x0: int, y0: int, x1: int, y1: int) -> np.ndarray:
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        if x0 < x1:
            sx = 1
        else:
            sx = -1
        if y0 < y1:
            sy = 1
        else:
            sy = -1
        err = dx - dy
        points = []
        while True:
            points.append([x0, y0])
            if x0 == x1 and y0 == y1:
                break
            e2 = 2 * err
            if e2 > -dy:
                err = err - dy
                x0 = x0 + sx
            if e2 < dx:
                err = err + dx
                y0 = y0 + sy
        return np.array(points)

    
    def laser_scan_to_polar(self, msg: LaserScan) -> np.ndarray:
        N = len(msg.ranges)
        array = np.zeros((N, 2))
        for i in range(len(msg.ranges)):
            angle = i * msg.angle_increment
            if msg.ranges[i] > msg.range_max:
                distance = msg.range_max
            elif msg.ranges[i] < msg.range_min:
                distance = msg.range_min
            else:
                distance = msg.ranges[i]
            array[i, 0] = distance
            array[i, 1] = angle
        return array

    def polar_to_cartesian(self, polar: np.ndarray, x: float, y: float, yaw: float) -> np.ndarray:
        cartesian = np.zeros(polar.shape)
        for i in range(polar.shape[0]):
            cartesian[i, 0] = polar[i, 0] * np.cos(polar[i, 1] + yaw) + x
            cartesian[i, 1] = polar[i, 0] * np.sin(polar[i, 1] + yaw) + y
        return cartesian
    
    # def update_grid(self, points: np.ndarray):
    #     for point in points:
    #         x = int(point[0] / self.resolution)
    #         y = int(point[1] / self.resolution)
    #         if x >= 0 and x < self.grid.shape[0] and y >= 0 and y < self.grid.shape[1]:
    #             self.grid[x, y] = self.prob_occupied
    def publish_map(self, current_time: Time):
        print("publishing map")
        resolution = self.resolution
        world = (-self.width // 2, -self.height // 2)
        frame_id = "map"

        grid = OccupancyGrid()
        grid.header.stamp = current_time.to_msg()
        grid.header.frame_id = frame_id
        grid.info.resolution = resolution
        grid.info.width = self.grid.shape[0]
        grid.info.height = self.grid.shape[1]
        grid.info.origin.position.x = float(world[0])
        grid.info.origin.position.y = float(world[1])
        # grid data
        data = []
        for i in range(self.grid.shape[0]):
            for j in range(self.grid.shape[1]):
                data.append(int(100 * self.grid[i, j]))
        grid.data = data
        print("publishing map")
        self.publisher.publish(grid)
        




def main(args=None):
    rclpy.init()
    node = MyOccupancyGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

