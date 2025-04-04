#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import numpy as np
from geometry_msgs.msg import Pose, PoseArray, PoseStamped
from nav_msgs.msg import Odometry

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
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import Pose
from geometry_msgs.msg import Point
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Transform



from typing import Tuple

from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion
class State(Enum):
    FREE = 1
    OCCUPIED = 2
    UNKOWN = 3



def quaternion_from_euler(roll, pitch, yaw):
    """Convert Euler angles (roll, pitch, yaw) to a quaternion."""
    qx = roll / 2.0
    qy = pitch / 2.0
    qz = yaw / 2.0

    qw = np.cos(qx) * np.cos(qy) * np.cos(qz) + np.sin(qx) * np.sin(qy) * np.sin(qz)
    qx = np.sin(qx) * np.cos(qy) * np.cos(qz) - np.cos(qx) * np.sin(qy) * np.sin(qz)
    qy = np.cos(qx) * np.sin(qy) * np.cos(qz) + np.sin(qx) * np.cos(qy) * np.sin(qz)
    qz = np.cos(qx) * np.cos(qy) * np.sin(qz) - np.sin(qx) * np.sin(qy) * np.cos(qz)

    return [qx, qy, qz, qw]

class ParticleFilterNode(Node):
    def __init__(self):
        super().__init__('particle_filter_node')

        # Parameters
        self.num_particles = 1000
        self.particles = []
        self.weights = []
        self.robot_pose_odom = None
        self.robot_pose_odom_full = None

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
        self.count_grid = np.zeros(shape)

        # Map dimensions (for particle initialization)
        self.map_width = 20.0  # meters
        self.map_height = 20.0  # meters

        # Particle publisher
        self.particle_publisher = self.create_publisher(PoseArray, '/particles', 10)

        # Estimated pose publisher
        self.estimated_pose_publisher = self.create_publisher(PoseStamped, '/estimated_pose', 10)

        # Odometry subscriber
        self.odom_subscription = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)


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




        # Initialize particles
        self.initialize_particles()

        # Timer to publish particles
        self.create_timer(0.1, self.publish_particles)

    def initialize_particles(self):
        """Initialize particles randomly within the map."""
        self.particles = []
        self.weights = []
        for _ in range(self.num_particles):
            x = np.random.uniform(-self.map_width / 2, self.map_width / 2)
            y = np.random.uniform(-self.map_height / 2, self.map_height / 2)
            yaw = np.random.uniform(0, 2 * np.pi)
            self.particles.append((x, y, yaw))
            self.weights.append(1.0 / self.num_particles)  # Uniform weights

    def odom_callback(self, msg):
        """Update particles based on odometry data."""
        current_pose = msg.pose.pose
        self.robot_pose_odom_full = current_pose

        if self.robot_pose_odom is not None:
            # Calculate motion (delta_x, delta_y, delta_yaw)
            prev_x, prev_y, prev_yaw = self.robot_pose_odom
            curr_x = current_pose.position.x
            curr_y = current_pose.position.y
            _, _, curr_yaw = self.euler_from_quaternion(current_pose.orientation)

            delta_x = curr_x - prev_x
            delta_y = curr_y - prev_y
            delta_yaw = curr_yaw - prev_yaw

            # Predict particle motion
            self.predict_particles(delta_x, delta_y, delta_yaw)

        # Update the robot's pose
        self.robot_pose_odom = (
            current_pose.position.x,
            current_pose.position.y,
            self.euler_from_quaternion(current_pose.orientation)[2],
        )

        # assign the values
        # self.robot_pose_odom_full.position.x = self.robot_pose_odom[0]
        # self.robot_pose_odom_full.position.y = self.robot_pose_odom[1]
        # self.robot_pose_odom_full.orientation.z = self.robot_pose_odom[2]

        # Estimate the robot's position
        estimated_pose = self.estimate_pose()

        self.robot_pose_odom_full.position.x = estimated_pose[0]
        self.robot_pose_odom_full.position.y = estimated_pose[1]
        self.robot_pose_odom_full.orientation.z = estimated_pose[2]



        self.publish_estimated_pose(estimated_pose)

    def predict_particles(self, delta_x, delta_y, delta_yaw):
        """Update particle positions based on motion."""
        new_particles = []
        for x, y, yaw in self.particles:
            # Apply motion model with noise
            new_x = x + delta_x + np.random.normal(0, 0.1)  # Add noise
            new_y = y + delta_y + np.random.normal(0, 0.1)
            new_yaw = yaw + delta_yaw + np.random.normal(0, 0.05)
            new_particles.append((new_x, new_y, new_yaw))
        self.particles = new_particles

    def estimate_pose(self):
        """Estimate the robot's pose as the weighted average of the particles."""
        x = sum(p[0] * w for p, w in zip(self.particles, self.weights))
        y = sum(p[1] * w for p, w in zip(self.particles, self.weights))
        yaw = sum(p[2] * w for p, w in zip(self.particles, self.weights))
        return x, y, yaw+np.pi

    def publish_particles(self):
        """Publish the particle cloud as a PoseArray."""
        pose_array = PoseArray()
        pose_array.header.frame_id = "map"
        pose_array.header.stamp = self.get_clock().now().to_msg()

        for x, y, yaw in self.particles:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = 0.0
            q = quaternion_from_euler(0, 0, yaw)
            pose.orientation.x = q[0]
            pose.orientation.y = q[1]
            pose.orientation.z = q[2]
            pose.orientation.w = q[3]
            pose_array.poses.append(pose)

        self.particle_publisher.publish(pose_array)

    def publish_estimated_pose(self, estimated_pose):
        """Publish the estimated pose as a PoseStamped."""
        x, y, yaw = estimated_pose
        pose_stamped = PoseStamped()
        pose_stamped.header.frame_id = "map"
        pose_stamped.header.stamp = self.get_clock().now().to_msg()
        pose_stamped.pose.position.x = x
        pose_stamped.pose.position.y = y
        pose_stamped.pose.position.z = 0.0
        q = quaternion_from_euler(0, 0, yaw)
        pose_stamped.pose.orientation.x = q[0]
        pose_stamped.pose.orientation.y = q[1]
        pose_stamped.pose.orientation.z = q[2]
        pose_stamped.pose.orientation.w = q[3]

        # print the pose
        self.get_logger().info(f"Estimated Pose: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}")

        self.estimated_pose_publisher.publish(pose_stamped)

    def euler_from_quaternion(self, quaternion):
        """Convert a quaternion to Euler angles (roll, pitch, yaw)."""
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

    def scan_callback(self, msg):
        # get the current time
        current_time = self.get_clock().now()
        self.scan_has_been_received = True
        self.get_logger().info("Updated occupancy grid...")

        if(self.robot_pose_odom_full is None):
            self.get_logger().info("Robot pose not received yet")
            return

        # now we can use the robot pose to transfor the laser scan an update the grid
        # _, _, yaw = self.euler_from_quaternion(self.robot_pose_odom_full.orientation)
        yaw = self.robot_pose_odom_full.orientation.z
        if(yaw < 0):
            yaw += 2 * np.pi
        # get the laser scan in polar coordinates
        polar = self.laser_scan_to_polar(msg)
        # transform the laser scan to cartesian coordinates
        xy = self.polar_to_cartesian(polar, self.robot_pose_odom_full.position.x, self.robot_pose_odom_full.position.y, yaw)
        xy[:, 0] = (xy[:, 0] + self.width // 2) / self.resolution
        xy[:, 1] = (xy[:, 1] + self.height // 2) / self.resolution
        xy = xy.astype(int)

        
        xo = int((self.robot_pose_odom_full.position.x + self.width // 2) / self.resolution)
        yo = int((self.robot_pose_odom_full.position.y + self.height // 2) / self.resolution)
        # update the grid
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

        self.publish_map(current_time)
        print("Map published")
            


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
        # update the cell in the grid with a counter instead of a probability
        if(state == State.FREE):
            self.grid[x, y] = 0
        elif(state == State.OCCUPIED):
            self.grid[x, y] += 10
        else:
            self.grid[x, y] = 0

        # clamp the values between 0 and 100
        if(self.grid[x, y] < 0):
            self.grid[x, y] = 0
        if(self.grid[x, y] > 100):
            self.grid[x, y] = 100



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
        copy_grid[:, :] = self.grid[:, :]
        copy_grid = np.transpose(copy_grid)
        copy_grid = copy_grid.astype("int8")
        grid.data = copy_grid.ravel().tolist()

        print("publishing map")
        self.publisher.publish(grid)


def main(args=None):
    rclpy.init(args=args)
    node = ParticleFilterNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()