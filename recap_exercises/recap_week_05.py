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
def euler_from_quaternion(quaternion: Quaternion):
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


def laser_scan_to_polar(message: LaserScan) -> np.ndarray:
    N = len(message.ranges)
    array = np.zeros((N, 2))
    for i in range(len(message.ranges)):
        angle = i * message.angle_increment
        if message.ranges[i] > message.range_max:
            distance = message.range_max
        elif message.ranges[i] < message.range_min:
            distance = message.range_min
        else:
            distance = message.ranges[i]
        array[i, 0] = distance
        array[i, 1] = angle
    return array


def polar_to_cartesian(coordinates: np.ndarray, x: float, y: float, theta: float) -> np.ndarray:
    N = coordinates.shape[0]
    array = np.zeros((N, 2))
    array[:, 0] = x + coordinates[:, 0] * np.cos(coordinates[:, 1] + theta)
    array[:, 1] = y + coordinates[:, 0] * np.sin(coordinates[:, 1] + theta)
    return array


def numpy_to_occupancy_grid(array: np.ndarray, resolution: float, world: Tuple[float, float], frame_id: str, timestamp: Time):
    if not len(array.shape) == 2:
        raise TypeError('Array must be 2D')
    if not array.dtype == np.int8:
        raise TypeError('Array must be of int8s')
    grid = OccupancyGrid()
    if isinstance(array, np.ma.MaskedArray):
        array = array.data
    grid.header.stamp = timestamp
    grid.header.frame_id = frame_id
    grid.data = array.ravel().tolist()
    grid.info.resolution = resolution
    grid.info.height = array.shape[0]
    grid.info.width = array.shape[1]
    grid.info.origin.position.x = float(world[0])
    grid.info.origin.position.y = float(world[1])
    return grid


def occupancy_grid_to_numpy(msg):
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    return data

class MyOccupancyGrid(Node):
    def __init__(self, what_function_to_call_on_scan):
        super().__init__('my_occupancy_grid')
        # initilize so we do not publish the map before the first scan
        self.scan_has_been_received = False
        # define the grid
        self.width = float(100)
        self.height = float(100)
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
        self.grid = np.ones(shape) * self.prob_priori
        self.counter_grid = np.zeros(shape, dtype=int)  # Initialize the counter grid
        
        # subcription to the laser scan        
        self.laser_subscription = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.laser_subscription  # prevent unused variable warning
        # publisher to map in order to publish the grid map
        self.publisher_map = self.create_publisher(OccupancyGrid, '/map_2', 10)
        self.publisher_map
        # timer to publish and update the map
        self.timer = self.create_timer(1, self.timer_callback)

        # set up the transform buffer
        self._tf_buffer = Buffer()                                          # set the transform buffer
        self._tf_listener = TransformListener(self._tf_buffer, self)        # set the transform listener
        self._tf_publisher = StaticTransformBroadcaster(self)           # set the transform broadcaster
        # transform from map to odom
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self._tf_publisher.sendTransform(tf)

        self.what_function_to_call_on_scan = what_function_to_call_on_scan

    def scan_callback(self, msg):
        if(self.what_function_to_call_on_scan == "update_map"):
            self.update_map(msg)
            return
        
        if(self.what_function_to_call_on_scan == "update_map_with_counter"):
            self.update_map_with_counter(msg)
            return

        print("what_function_to_call_on_scan is not set to a valid value")
        exit()

    def update_map(self, msg):
        print("update_map")
        self.scan_has_been_received = True
        self.get_logger().info("Updated occupancy grid...")
        # transform between the laser scan and the odom frame
        try:
            tf = self._tf_buffer.lookup_transform('odom', msg.header.frame_id, msg.header.stamp)
        except Exception as e:
            self.get_logger().error('Could not transform between odom and %s: %s' % (msg.header.frame_id, str(e)))
            return
        _, _, yaw = euler_from_quaternion(tf.transform.rotation)
        if yaw < 0:
            yaw += 2 * np.pi
        # convert the laser scan to polar coordinates
        polar = laser_scan_to_polar(msg)
        # we pass the tf.transform.translation.x and tf.transform.translation.y to the function polar_to_cartesian
        # because those give use the position of the laser scan in the odom frame
        # and we want the odom fraom
        xy = polar_to_cartesian(polar, tf.transform.translation.x, tf.transform.translation.y, yaw)
        xy[:, 0] = (xy[:, 0] + self.width // 2) / self.resolution
        xy[:, 1] = (xy[:, 1] + self.height // 2) / self.resolution
        xy = xy.astype(int)
        xo = int((tf.transform.translation.x + self.width // 2) / self.resolution)
        yo = int((tf.transform.translation.y + self.height // 2) / self.resolution)
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
                self.update_cell(x=x - 1, y=y, state=State.OCCUPIED)
                self.update_cell(x=x + 1, y=y, state=State.OCCUPIED)
                self.update_cell(x=x, y=y + 1, state=State.OCCUPIED)
                self.update_cell(x=x, y=y - 1, state=State.OCCUPIED)
            else:
                self.update_cell(x=x, y=y, state=State.FREE)
        self.get_logger().info("Updated occupancy grid...")
    def update_map_2(self, pose):
        self.scan_has_been_received = True
        self.get_logger().info("Updated occupancy grid with odometry data...")
        
        # Extract position and orientation from the pose
        position = pose.position
        orientation = pose.orientation
        
        # Convert orientation from quaternion to Euler angles
        _, _, yaw = euler_from_quaternion(orientation)
        if yaw < 0:
            yaw += 2 * np.pi
        
        # Convert the position to grid coordinates
        x = int((position.x + self.width // 2) / self.resolution)
        y = int((position.y + self.height // 2) / self.resolution)
        
        # Update the occupancy grid based on the current position
        self.update_cell(x=x, y=y, state=State.OCCUPIED)
        self.update_cell(x=x - 1, y=y, state=State.OCCUPIED)
        self.update_cell(x=x + 1, y=y, state=State.OCCUPIED)
        self.update_cell(x=x, y=y + 1, state=State.OCCUPIED)
        self.update_cell(x=x, y=y - 1, state=State.OCCUPIED)
        
        self.get_logger().info(f"Updated map with position: {position} and orientation: {orientation}")
    def timer_callback(self):
        if(self.scan_has_been_received):
            self.publisher_map.publish(self.my_occupancy_grid)

    def update_map_with_counter(self, msg):
        print("update_map_with_counter")
        self.scan_has_been_received = True
        self.get_logger().info("Updated occupancy grid...")
        # transform between the laser scan and the odom frame
        try:
            tf = self._tf_buffer.lookup_transform('odom', msg.header.frame_id, msg.header.stamp)
        except Exception as e:
            self.get_logger().error('Could not transform between odom and %s: %s' % (msg.header.frame_id, str(e)))
            return
        _, _, yaw = euler_from_quaternion(tf.transform.rotation)
        if yaw < 0:
            yaw += 2 * np.pi
        # convert the laser scan to polar coordinates
        polar = laser_scan_to_polar(msg)
        # we pass the tf.transform.translation.x and tf.transform.translation.y to the function polar_to_cartesian
        # because those give use the position of the laser scan in the odom frame
        # and we want the odom fraom
        xy = polar_to_cartesian(polar, tf.transform.translation.x, tf.transform.translation.y, yaw)
        xy[:, 0] = (xy[:, 0] + self.width // 2) / self.resolution
        xy[:, 1] = (xy[:, 1] + self.height // 2) / self.resolution
        xy = xy.astype(int)
        xo = int((tf.transform.translation.x + self.width // 2) / self.resolution)
        yo = int((tf.transform.translation.y + self.height // 2) / self.resolution)
        for i in range(xy.shape[0]):
            points = self.bresenham(x1=xo, y1=yo, x2=xy[i, 0], y2=xy[i, 1])
            for j in range(points.shape[0] - 1):
                x = points[j, 0]
                y = points[j, 1]
                self.update_cell_with_counter(x=x, y=y, state=State.FREE)
            x = xy[i, 0]
            y = xy[i, 1]
            if polar[i, 0] < msg.range_max:
                self.update_cell_with_counter(x=x, y=y, state=State.OCCUPIED)
                self.update_cell_with_counter(x=x - 1, y=y, state=State.OCCUPIED)
                self.update_cell_with_counter(x=x + 1, y=y, state=State.OCCUPIED)
                self.update_cell_with_counter(x=x, y=y + 1, state=State.OCCUPIED)
                self.update_cell_with_counter(x=x, y=y - 1, state=State.OCCUPIED)
            else:
                self.update_cell_with_counter(x=x, y=y, state=State.FREE)
        self.get_logger().info("Updated occupancy grid...")

    def update_cell_with_counter(self, x: int, y: int, state: State):
        """Update the cell using a counter-based approach."""
        if state == State.FREE:
            self.counter_grid[x, y] -= 1
        elif state == State.OCCUPIED:
            self.counter_grid[x, y] += 1

        # Clamp the counter value between 0 and 100
        if self.counter_grid[x, y] < 0:
            self.counter_grid[x, y] = 0
        elif self.counter_grid[x, y] > 100:
            self.counter_grid[x, y] = 100

        # Update the occupancy probability based on the counter value
        self.grid[x, y] = self.counter_grid[x, y]/100


    @property
    def my_occupancy_grid(self):
        # copy data from grid
        grid = np.zeros_like(self.grid)
        # multiple by 100, to give probabilities between 0 and 100
        grid[:, :] = self.grid[:, :] * 100
        # grid = np.rot90(grid)
        grid = np.transpose(grid)
        # convert to int8 and returns as occupancy grid
        grid = grid.astype("int8")
        return numpy_to_occupancy_grid(array=grid, resolution=self.resolution, world=(-self.width // 2, -self.height // 2), frame_id="map", timestamp=self.get_clock().now().to_msg())


    def update_cell(self, x: int, y: int, state: State):
        """Given a state observed given laser measurements, update occupancy grid using log-odds
        and retrive probability to be store at this position.

        Parameters
        ----------
        x: int
            X-axis position in occupancy grid.
        y: int
            Y-axis position in occupancy grid.
        state: :py:class:`State <enum.state>`
            Flag used to indicate the status observed at this position in the occupancy grid.
        """
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
    @staticmethod
    def log_odd(probability: float) -> float:
        """A log odds in statistics is the logarithm of the odds ratio. The advantage of the
        log-odds over the probability representation is that we can avoid numerical instabilities
        for probabilities near zero or one.

        Parameters
        ----------
        probability: float
            Given probability to estimate log-odds.

        Returns
        -------
        float
            Return log-odds of a given probability.
         """
        return np.log(probability / (1.0 - probability))

    @staticmethod
    def probability(log_odd: float) -> float:
        """Retrieve probability from a log-odds representation. It will always be a value between 0
        and 1.

        Parameters
        ----------
        log_odd: float
            Given log-odd to estimate probability.

        Returns
        -------
        float
            Return probability between 0 and 1.
        """
        result = 1 - (1.0 / (1 + np.exp(log_odd)))
        if np.isnan(result):
            result = 0.0
        return result
    @staticmethod
    def bresenham(x1: int, y1: int, x2: int, y2: int) -> np.ndarray:
        """Implementation of Bresenham's line drawing algorithm. The first point can be thought of
        as the source point and the second point as the target point.

        Parameters
        ----------
        x1: int
            X-axis position in occupancy grid of first point.
        y1: int
            Y-axis position in occupancy grid of first point.
        x2: int
            X-axis position in occupancy grid of second point.
        y2: int
            Y-axis position in occupancy grid of second point.
        Returns
        -------
        :py:class:`ndarray <numpy.ndarray>` of shape `(N,2)`
            The first column corresponds to the X's and the second column corresponds to the Y's
            occupancy grid indexes that the line covers.

        Examples
        --------
        >>> points1 = bresenham((4, 4), (6, 10))
        >>> print(points1)
        np.array([[4,4], [4,5], [5,6], [5,7], [5,8], [6,9], [6,10]])

        Reference
        ---------
        Code inspired  by a collection of sample codes for robotics algorithms.
        Source code at:
        * https://github.com/AtsushiSakai/PythonRobotics
        """
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



def main(args=None):
    rclpy.init()
    node = MyOccupancyGrid("update_map")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    

