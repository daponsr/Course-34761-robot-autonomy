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
        self.width = 20
        self.height = 20
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
        self.grid = np.ones(shape) * (-1)

        # subscription to the lidar data
        self.subscription = self.create_subscription(LaserScan,'/scan',self.scan_callback,10)
        self.subscription

        # publisher for the map --> where we will publish the map (OccupancyGrid)
        self.publisher = self.create_publisher(OccupancyGrid, '/map_3', 10)
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
        self.scan_has_been_received = True
        self.get_logger().info("Updated occupancy grid...")
        # transform between the laser scan and the odom frame
        
        try:
            print("transofrm",'odom', msg.header.frame_id)
            transform = self.tf_buffer.lookup_transform('odom', msg.header.frame_id, msg.header.stamp)
        except Exception as e:
            self.get_logger().error('Could not transform between odom and %s: %s' % (msg.header.frame_id, str(e)))
            return
        _, _, yaw = self.euler_from_quaternion(transform.transform.rotation)
        if yaw < 0:
            yaw += 2 * np.pi
        # convert the laser scan to polar coordinates
        polar = self.laser_scan_to_polar(msg)
        # we pass the transform.transform.translation.x and transform.transform.translation.y to the function polar_to_cartesian
        # because those give use the position of the laser scan in the odom frame
        # and we want the odom fraom
        xy = self.polar_to_cartesian(polar, transform.transform.translation.x, transform.transform.translation.y, yaw)
        xy[:, 0] = (xy[:, 0] + self.width // 2) / self.resolution
        xy[:, 1] = (xy[:, 1] + self.height // 2) / self.resolution
        xy = xy.astype(int)
        xo = int((transform.transform.translation.x + self.width // 2) / self.resolution)
        yo = int((transform.transform.translation.y + self.height // 2) / self.resolution)
        for i in range(xy.shape[0]):
            points = self.bresenham(x1=xo, y1=yo, x2=xy[i, 0], y2=xy[i, 1])
            for j in range(points.shape[0] - 1):
                x = points[j, 0]
                y = points[j, 1]
                self.update_cell(x=x, y=y, state=State.FREE)
            x = xy[i, 0]
            y = xy[i, 1]
            if polar[i, 0] < msg.range_max:
                self.update_cell(x=x, y=y, state=State.OCCUPIED)
            else:
                self.update_cell(x=x, y=y, state=State.FREE)
        self.get_logger().info("Updated occupancy grid...")

        # publish the map
        self.publish_map(current_time)
        print("publish my grid")

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
    

    def update_cell(self, x: int, y: int, state: State):
        if state == State.FREE:
            log_prob = self.log_odd(probability=self.prob_free)
        elif state == State.OCCUPIED:
            log_prob = self.log_odd(probability=self.prob_occupied)
        else:
            log_prob = self.log_odd(probability=self.prob_priori)
        current_prob = self.grid[x, y]
        current_prob_log_odd = self.log_odd(probability=current_prob)
        current_prob_log_odd += log_prob
        new_prob = self.probability(log_odd=current_prob_log_odd)
        if new_prob < self.min_prob:
            new_prob = self.min_prob
        elif new_prob > self.max_prob:
            new_prob = self.max_prob
        self.grid[x, y] = new_prob

    def log_odd(self, probability: float) -> float:
        return np.log(probability / (1 - probability))
    def probability(self, log_odd: float) -> float:
        result = 1 - (1.0 / (1 + np.exp(log_odd)))
        if np.isnan(result):
            result = 0.0
        return result

    def bresenham(self, x1: int, y1: int, x2: int, y2: int) -> np.ndarray:
        # setup initial conditions
        dx = x2 - x1
        dy = y2 - y1
        # determine how steep the line is
        is_steep = abs(dy) > abs(dx)
        # rotate line
        if is_steep:
            x1, y1 = y1, x1
            x2, y2 = y2, x2
        # swap start and end points if necessary and store swap state
        swapped = False
        if x1 > x2:
            x1, x2 = x2, x1
            y1, y2 = y2, y1
            swapped = True
        # recalculate differentials
        dx = x2 - x1
        # recalculate differentials
        dy = y2 - y1
        # calculate error
        error = int(dx / 2.0)
        y_step = 1 if y1 < y2 else -1
        # iterate over bounding box generating points between start and end
        y = y1
        points = []
        for x in range(x1, x2 + 1):
            coord = [y, x] if is_steep else (x, y)
            points.append(coord)
            error -= abs(dy)
            if error < 0:
                y += y_step
                error += dx
        # reverse the list if the coordinates were swapped
        if swapped:
            points.reverse()
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


        copy_grid = np.zeros_like(self.grid)
        copy_grid[:, :] = self.grid[:, :] * 100
        copy_grid = np.transpose(copy_grid)
        copy_grid = copy_grid.astype("int8")
        grid.data = copy_grid.ravel().tolist()

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
    

