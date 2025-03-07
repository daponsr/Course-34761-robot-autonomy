
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

from typing import Tuple

import numpy as np

from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion


def euler_from_quaternion(quaternion: Quaternion):
    """ Convert quaternion (w in last place) to euler roll, pitch, yaw.

    Parameters
    ----------
    quaternion: :py:class:`Quarternion <geometry_msgs.msg._quaternion.Quaternion>`
        Orientation as a quaternion.

    Returns
    -------
    Tuple[float,float,float]
        Return roll, pitch, yaw euler angles.

    References
    ----------
    Source: https://github.com/ROBOTIS-GIT/turtlebot3/blob/humble-devel/turtlebot3_example
    """
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
    """Convert a LaserScan message to a numpy array with points polar coordinates obtained. If
    the message indicates infinity, set the distance as the sensor's maximum value. In the
    same way, if the message indicates a distance value smaller than the minimum distance, set
    the sensor's minimum distance.

    Parameters
    ----------
    message: :py:class:`LaserScan <sensor_msgs.msg._laser_scan.LaserScan>`
        Message received from laser sensor.

    Returns
    -------
    :py:class:`ndarray <numpy.ndarray>` of shape `(N,2)`
        Points sampled in polar coordinates. The first column corresponds to the distance (in
        meters) and the second column corresponds to the angle (in radians).
    """
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
    """Convert points in polar coordinates in the laser reference to cartesian coordinates in
    the odometry reference.

    Parameters
    ----------
    coordinates: :py:class:`ndarray <numpy.ndarray>` of shape `(N,2)`
        Points sampled in polar coordinates. The first column corresponds to the distance (in
        meters) and the second column corresponds to the angle (in radians).
    x: float
        Laser position on the X-axis in the odometry referential.
    y: float
        Laser position on the Y-axis in the odometry referential.
    theta: float
        Laser rotation in the odometry referential.

    Returns
    -------
    :py:class:`ndarray <numpy.ndarray>` of shape `(N,2)`
        Points sample in cartesian coordinates at odometry referential. The first column
        corresponds to the X's and the second column corresponds to the Y's.
    """
    N = coordinates.shape[0]
    array = np.zeros((N, 2))
    array[:, 0] = x + coordinates[:, 0] * np.cos(coordinates[:, 1] + theta)
    array[:, 1] = y + coordinates[:, 0] * np.sin(coordinates[:, 1] + theta)
    return array


def numpy_to_occupancy_grid(array: np.ndarray, resolution: float, world: Tuple[float, float],
                            frame_id: str, timestamp: Time):
    """Utility to converts an :py:class:`ndarray <numpy.ndarray>` to an
    :py:class:`OccupancyGrid <nav_msgs.msg._occupancy_grid.OccupancyGrid>`.

    Parameters
    ----------
    array: :py:class:`ndarray <numpy.ndarray>` of shape `(N,M)`
        Two dimentional numpy array to convert in a occupancy grid message.
    resolution: float
        Occupancy grid resolution.
    world: Tuple[float,float]
        `(x,y)` world position.
    frame_id: str
        Referential frame name.
    timestamp: Time
        Timestamp info.

    Returns
    -------
    :py:class:`OccupancyGrid <nav_msgs.msg._occupancy_grid.OccupancyGrid>`
        Occupancy grid message.

    Reference
    ---------
    Code inspired by `ros_numpy` package, a collection of conversion function for extracting
    numpy arrays from messages. Source code at:
    * http://docs.ros.org/en/jade/api/ros_numpy/html/occupancy__grid_8py_source.html
    """
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
    """
    Source: http://docs.ros.org/en/jade/api/ros_numpy/html/occupancy__grid_8py_source.html
    """
    data = np.asarray(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
    return data