#!/usr/bin/env python3
import os
import rclpy
from nav2_msgs.msg import ParticleCloud  # Ensure correct import
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import numpy as np
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException, TransformException
from tf2_ros.transform_listener import TransformListener
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose, Point, Quaternion

class ParticleCloudSubscriber(Node):
    def __init__(self):
        super().__init__('listener_cloud_sub')
        
        self.particle_count = 0
        self.odom_pose = None
        self.particle_pose = None
        self.pose_from_tf = None
        self.amcl_pose = None



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



        # Define QoS settings
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.particle_cloud_sub = self.create_subscription(
            ParticleCloud, 
            "/particle_cloud", 
            self.particle_cloud_callback, 
            qos_profile
        )
        
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            "/amcl_pose", 
            self.amcl_pose_callback, 
            qos_profile
        )

        # subscribe to the odometry topic
        self.amcl_pose_sub = self.create_subscription(
            Odometry, 
            "/odom", 
            self.odom_callback, 
            qos_profile
        )

        # add a time callback
        self.create_timer(1.0, self.timer_callback)
        
        print("Subscribed to /particle_cloud and /amcl_pose")



    
    def odom_callback(self, msg):
        # save the odometry pose
        self.odom_pose = msg.pose.pose
        
        # get the transform from "map" to "odom"
        try:
            transform = self.tf_buffer.lookup_transform("map", "base_link", rclpy.time.Time())
            self.pose_from_tf = transform.transform.translation
        except (LookupException, TransformException) as e:
            self.get_logger().error(f"Transform error: {e}")
            return


    def timer_callback(self):
        str_print = ""
        # early return if any of the pose is None
        if self.particle_pose != None:
            str_print += f"Particle pose:  {self.particle_pose[0]:2.2f}  {self.particle_pose[1]:2.2f}\n"
        if self.amcl_pose != None:
            str_print += f"AMCL pose:      {self.amcl_pose.position.x:2.2f}  {self.amcl_pose.position.y:2.2f}\n"
        if self.odom_pose != None:
            str_print += f"Odometry pose:  {self.odom_pose.position.x:2.2f}  {self.odom_pose.position.y:2.2f}\n"
        if self.pose_from_tf != None:
            str_print += f"Pose from tf:   {self.pose_from_tf.x:2.2f}  {self.pose_from_tf.y:2.2f}\n"
        if self.particle_count != None:
            str_print += f"Particle count: {self.particle_count}\n"

        # print the string
        print(str_print)


    def particle_cloud_callback(self, msg):
        print("amount of particles", len(msg.particles))
        self.particle_count = len(msg.particles)
        positions = [(particle.pose.position.x, particle.pose.position.y) for particle in msg.particles]
        self.particle_count = len(msg.particles)
        positions = [(particle.pose.position.x, particle.pose.position.y) for particle in msg.particles]
        
        # Compute the weighted average position
        x_sum = sum(p[0] for p in positions)
        y_sum = sum(p[1] for p in positions)
        num_particles = len(positions)
        
        if num_particles > 0:
            x_avg = x_sum / num_particles
            y_avg = y_sum / num_particles
        else:
            print("No particles received")
            return
        
        # Take the real x and y position that is closer to the average position
        min_x_dist = float('inf')
        min_y_dist = float('inf')
        real_x = float('inf')
        real_y = float('inf')

        for particle in msg.particles:
            x_dist = abs(particle.pose.position.x - x_avg)
            y_dist = abs(particle.pose.position.y - y_avg)
            if x_dist < min_x_dist:
                min_x_dist = x_dist
                real_x = particle.pose.position.x
            if y_dist < min_y_dist:
                min_y_dist = y_dist
                real_y = particle.pose.position.y

        # particle pose is the average of all particles
        self.particle_pose = (real_x, real_y)

    
    def amcl_pose_callback(self, msg):
        # Extract the estimated position from the AMCL pose
        self.amcl_pose = msg.pose.pose


def main():
    rclpy.init()
    node = ParticleCloudSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    print("recap_week_06.py")
    main()