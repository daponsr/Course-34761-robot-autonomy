#!/usr/bin/env python3
import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import TransformStamped
from tf2_ros.transform_broadcaster import TransformBroadcaster

from nav_msgs.msg import Odometry  # Import Odometry message

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__('lidar_subscriber')
        # Subscribe to Lidar scan
        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

        # Subscribe to /odom topic
        self.odom_subscription = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10)
        self.odom_subscription  # prevent unused variable warning

        self.past_cloud = []
        self.tf_broadcaster = TransformBroadcaster(self)
        self.current_transform = np.eye(3)  # Initialize transformation matrix as identity
        self.constant_velocity = 0.05  # Constant velocity in m/s
        self.last_time = self.get_clock().now()  # Track the last time step

        self.odom_pose = None  # Store the ground truth pose from /odom

        self.first_odom_received = False
        self.initial_pose = False
        self.initial_pose_vals = None

    def odom_callback(self, msg):
        # Extract position and orientation from /odom
        position = msg.pose.pose.position
        orientation = msg.pose.pose.orientation

        




        # Convert quaternion to yaw
        siny_cosp = 2 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1 - 2 * (orientation.y**2 + orientation.z**2)
        yaw = math.atan2(siny_cosp, cosy_cosp)

        # Store the ground truth pose
        self.odom_pose = {
            'x': position.x,
            'y': position.y,
            'yaw': yaw
        }

        self.first_odom_received = True

    def listener_callback(self, msg):

        if not self.first_odom_received:
            self.get_logger().warn("No ground truth pose available from /odom yet.")
            return
        if not self.initial_pose:
            self.initial_pose = True
            self.last_time = self.get_clock().now()
            self.initial_pose_vals = self.odom_pose
            print("Initial pose set")



        # print("Received Lidar scan")
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds * 1e-9  # Time difference in seconds
        self.last_time = current_time

        # Convert Lidar ranges to Cartesian points
        ranges = self.filter_invalid_points(msg.ranges)
        cartesian_points = []
        for i in range(len(ranges)):
            angle = msg.angle_min + i * msg.angle_increment
            x = ranges[i] * math.cos(angle)
            y = ranges[i] * math.sin(angle)
            # take the starting point as the initial pose
            cartesian_points.append([x, y])



        # Predict transformation using constant velocity
        predicted_transform = self.predict_transform(dt)

        # Perform ICP to compute relative transformation
        if len(self.past_cloud) > 0:
            icp_transform = self.perform_icp(self.past_cloud, cartesian_points)
            self.current_transform = np.dot(self.current_transform, np.dot(predicted_transform, icp_transform))  # Combine prediction and ICP
            self.publish_tf(self.current_transform)

            # Compare with ground truth from /odom
            self.compare_with_odom()

        # Update past cloud
        self.past_cloud = cartesian_points

    def compare_with_odom(self):
        print("compare_with_odom")
        if self.odom_pose is None:
            self.get_logger().warn("No ground truth pose available from /odom yet.")
            return

        # Extract computed pose from the transformation matrix
        computed_x = self.current_transform[0, 2]
        computed_y = self.current_transform[1, 2]
        computed_yaw = math.atan2(self.current_transform[1, 0], self.current_transform[0, 0])

        # Extract ground truth pose
        ground_truth_x = self.odom_pose['x']
        ground_truth_y = self.odom_pose['y']
        ground_truth_yaw = self.odom_pose['yaw']

        # Compute differences
        dx = ground_truth_x - computed_x
        dy = ground_truth_y - computed_y
        dyaw = ground_truth_yaw - computed_yaw

        # Log the differences
        self.get_logger().info(f"Pose Comparison:")
        self.get_logger().info(f"Ground Truth: x={ground_truth_x:.2f}, y={ground_truth_y:.2f}, yaw={ground_truth_yaw:.2f}")
        self.get_logger().info(f"Computed: x={computed_x:.2f}, y={computed_y:.2f}, yaw={computed_yaw:.2f}")
        self.get_logger().info(f"Differences: dx={dx:.2f}, dy={dy:.2f}, dyaw={dyaw:.2f}")

    # Other methods (filter_invalid_points, predict_transform, perform_icp, compute_covariance, publish_tf) remain unchanged

    def filter_invalid_points(self, cloud):
        # Filter out invalid points
        filtered_cloud = []
        for i in range(len(cloud)):
            if cloud[i] != float('Inf'):
                filtered_cloud.append(cloud[i])
            else:
                filtered_cloud.append(0)
        return filtered_cloud


    def predict_transform(self, dt):
        # Predict transformation based on constant velocity
        dx = self.constant_velocity * dt  # Distance traveled in x-direction
        return np.array([
            [1, 0, dx],  # Translation in x
            [0, 1, 0],   # No translation in y
            [0, 0, 1]    # Homogeneous transformation
        ])

    def perform_icp(self, cloud1, cloud2):
        # Compute covariance matrix
        cov = self.compute_covariance(cloud1, cloud2)
        U, S, Vt = np.linalg.svd(cov)
        R = np.dot(Vt.T, U.T)
        t = np.dot(-R, np.mean(cloud2, axis=0)) + np.mean(cloud1, axis=0)
        transformation_matrix = np.vstack([np.column_stack([R, t]), [0, 0, 1]])
        return transformation_matrix

    def compute_covariance(self, cloud1, cloud2):
        centroid1 = np.mean(cloud1, axis=0)
        centroid2 = np.mean(cloud2, axis=0)
        centered_cloud1 = cloud1 - centroid1
        centered_cloud2 = cloud2 - centroid2
        return np.dot(centered_cloud1.T, centered_cloud2)

    def publish_tf(self, transform):
        # Publish the transformation as a TF
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = transform[0, 2]
        t.transform.translation.y = transform[1, 2]
        t.transform.translation.z = 0.0
        rotation = np.arctan2(transform[1, 0], transform[0, 0])  # Extract rotation angle
        t.transform.rotation.x = 0.0
        t.transform.rotation.y = 0.0
        t.transform.rotation.z = math.sin(rotation / 2.0)
        t.transform.rotation.w = math.cos(rotation / 2.0)

        # publish
        print("Publishing TF")
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()