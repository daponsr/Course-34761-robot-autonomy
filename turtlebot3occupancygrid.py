
# Copyright 2022 Luiz Carlos Cosmi Filho and others.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import threading
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

from turtlebot3mapperutils import (
    numpy_to_occupancy_grid,
    euler_from_quaternion,
    laser_scan_to_polar,
    polar_to_cartesian,
)


class State(Enum):
    FREE = 1
    OCCUPIED = 2
    UNKOWN = 3


class Turtlebot3OccupancyGrid(Node):

    def parameter_callback(self, parameters: List[Parameter]) -> SetParametersResult:
        for param in parameters:
            if param.name == "min_prob":
                self.min_prob = param.value
                self.get_logger().info("Updated parameter min_prob=%.2f" % self.min_prob)
            elif param.name == "max_prob":
                self.max_prob = param.value
                self.get_logger().info("Updated parameter max_prob=%.2f" % self.max_prob)
            else:
                return SetParametersResult(successful=False)
        return SetParametersResult(successful=True)

    def __init__(self, node_name: str = 'turtlebot_occupancy_grid'):

        super(Turtlebot3OccupancyGrid, self).__init__(node_name=node_name)
        self.declare_parameters(
            namespace="",
            parameters=[
                (
                    "width",
                    20.0,
                    ParameterDescriptor(description="Map's width"),
                ),
                (
                    "height",
                    20.0,
                    ParameterDescriptor(description="Map's height"),
                ),
                (
                    "resolution",
                    0.01,
                    ParameterDescriptor(description="Map's resolution"),
                ),
                (
                    "min_prob",
                    0.01,
                    ParameterDescriptor(description="Map's min probability value"),
                ),
                (
                    "max_prob",
                    0.99,
                    ParameterDescriptor(description="Map's max probability value"),
                ),
                (
                    "prob_occupied",
                    0.6,
                    ParameterDescriptor(description="Occupied probability increment"),
                ),
                (
                    "prob_free",
                    0.4,
                    ParameterDescriptor(description="Free probability increment"),
                ),
                (
                    "prob_priori",
                    0.5,
                    ParameterDescriptor(description="Priori probability increment"),
                ),
                (
                    "rate",
                    1.0,
                    ParameterDescriptor(description="Update rate"),
                ),
            ],
        )
        self.width = float(self.get_parameter("width").value)
        self.height = float(self.get_parameter("height").value)
        self.resolution = float(self.get_parameter("resolution").value)
        self.min_prob = float(self.get_parameter("min_prob").value)
        self.max_prob = float(self.get_parameter("max_prob").value)
        self.prob_occupied = float(self.get_parameter("prob_occupied").value)
        self.prob_free = float(self.get_parameter("prob_free").value)
        self.prob_priori = float(self.get_parameter("prob_priori").value)
        self.rate = float(self.get_parameter("rate").value)
        self.add_on_set_parameters_callback(self.parameter_callback)
        # create 2D-array to holds occupancy grid
        N = int(1 / self.resolution)
        shape = (int(self.width * N), int(self.height * N))
        self.grid = self.prob_priori * np.ones(shape, dtype=float)
        # subscriber to receive laser messages
        # self._scan_subscriber = self.create_subscription(
        #     msg_type=LaserScan,
        #     topic="/scan",
        #     callback=self._scan_callback,
        #     qos_profile=10,
        # )
        self._scan_subscriber = self.create_subscription(LaserScan, "/scan", self._scan_callback, 10)
        # publisher to send probability map
        self._map_publisher = self.create_publisher(
            msg_type=OccupancyGrid,
            topic="/custom_map",
            qos_profile=10,
        )
        # timer to update map
        self._update_timer = self.create_timer(
            timer_period_sec=(1 / self.rate),
            callback=self._timer_callback,
        )
        self._scan_init = False
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._tf_publisher = StaticTransformBroadcaster(self)
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id = 'odom'
        tf.transform.translation.x = 0.0
        tf.transform.translation.y = 0.0
        tf.transform.translation.z = 0.0
        self._tf_publisher.sendTransform(tf)
        self._update = threading.Lock()
        self.get_logger().info(f"Init {node_name}")

    def _scan_callback(self, message: LaserScan):
        """Callback executed when node receives messages from laser scan in robot.

        Parameters
        ----------
        message_laser: :py:class:`LaserScan <sensor_msgs.msg._laser_scan.LaserScan>`
            message with laser scan received from robot.
        """
        self._scan_init = True
        self._scan = message
        self.update_map(message_laser=self._scan)

    def _timer_callback(self):
        """Timer callback used to publish occupancy grid. Only starts publishing after received
        message from laser scan in robot.
        """
        if self._scan_init:
            self._update.acquire()
            self._map_publisher.publish(self.occupancy_grid)
            self._update.release()
            self.get_logger().info("Sending occupancy grid...")

    def update_map(self, message_laser: LaserScan):
        """Utility look for transformation between laser scan frame and odometry frame, compute
        laser samples in odometry referential and mark as free or occupied. Note that when it finds
        an occupied sample, it also mark 4-components around it as occupied.

        Parameters
        ----------
        message_laser: :py:class:`LaserScan <sensor_msgs.msg._laser_scan.LaserScan>`
            message with laser scan received from robot.
        """
        print("update_map")
        if not self._update.locked():
            self._update.acquire()
            try:
                tf = self._tf_buffer.lookup_transform(
                    target_frame='odom',
                    source_frame=message_laser.header.frame_id,
                    time=message_laser.header.stamp,
                    timeout=Duration(seconds=(1 / (self.rate))),
                )
                quaternion = tf.transform.rotation
            except (TransformException, LookupException) as ex:
                self.get_logger().warn(
                    f'Could not transform "odom" to {message_laser.header.frame_id}: {ex}')
                self._update.release()
                return
            _, _, theta = euler_from_quaternion(quaternion=quaternion)
            if theta < 0.0:
                theta += 2 * np.pi
            polar = laser_scan_to_polar(message=message_laser)
            xy = polar_to_cartesian(
                coordinates=polar,
                x=tf.transform.translation.x,
                y=tf.transform.translation.y,
                theta=theta,
            )
            xy[:, 0] = (xy[:, 0] + self.width // 2) / self.resolution
            xy[:, 1] = (xy[:, 1] + self.height // 2) / self.resolution
            xy = xy.astype(int)
            xo = int((tf.transform.translation.x + self.width // 2) / self.resolution)
            yo = int((tf.transform.translation.y + self.height // 2) / self.resolution)
            for i in range(xy.shape[0]):
                points = self.bresenham(
                    x1=xo,
                    y1=yo,
                    x2=xy[i, 0],
                    y2=xy[i, 1],
                )
                for j in range(points.shape[0] - 1):
                    x = points[j, 0]
                    y = points[j, 1]
                    self.update_cell(x=x, y=y, state=State.FREE)
                x = xy[i, 0]
                y = xy[i, 1]
                if polar[i, 0] < message_laser.range_max:
                    self.update_cell(x=x, y=y, state=State.OCCUPIED)
                    self.update_cell(x=x - 1, y=y, state=State.OCCUPIED)
                    self.update_cell(x=x + 1, y=y, state=State.OCCUPIED)
                    self.update_cell(x=x, y=y + 1, state=State.OCCUPIED)
                    self.update_cell(x=x, y=y - 1, state=State.OCCUPIED)
                else:
                    self.update_cell(x=x, y=y, state=State.FREE)
            self.get_logger().info("Updated occupancy grid...")
            self._update.release()
        else:
            print("locked")

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

    @property
    def occupancy_grid(self):
        # copy data from grid
        grid = np.zeros_like(self.grid)
        # multiple by 100, to give probabilities between 0 and 100
        grid[:, :] = self.grid[:, :] * 100
        # grid = np.rot90(grid)
        grid = np.transpose(grid)
        # convert to int8 and returns as occupancy grid
        grid = grid.astype("int8")
        return numpy_to_occupancy_grid(
            array=grid,
            resolution=self.resolution,
            world=(-self.width // 2, -self.height // 2),
            frame_id="map",
            timestamp=self.get_clock().now().to_msg(),
        )

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