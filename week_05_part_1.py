import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header
import tf2_ros

UN_INIT_VAL = -1
OCCUPIED = 100
FREE = 0

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__("lidar_subscriber")

        # create the empty matrix
        self.MATRIX_RES = 0.1
        self.MATRIX_WIDTH_m = 15
        self.MATRIX_HEIGHT_m = 10
        self.MATRIX_DIV_W = int(self.MATRIX_WIDTH_m / self.MATRIX_RES)
        self.MATRIX_DIV_H = int(self.MATRIX_HEIGHT_m / self.MATRIX_RES)
        self.matrix = self.create_empty_matrix(self.MATRIX_DIV_W, self.MATRIX_DIV_H)
        self.populate_random_obstacles()

        self.publisher_ = self.create_publisher(OccupancyGrid, "/map_2", 10)
        self.timer = self.create_timer(2, self.timer_callback)

        # Initialize the static transform broadcaster
        self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self.publish_static_transform()

    def create_empty_matrix(self, rows, cols):
        return np.full((rows, cols), FREE)

    def populate_random_obstacles(self):
        num_obstacles = int(0.1 * self.MATRIX_DIV_W * self.MATRIX_DIV_H)  # 10% of the cells
        for _ in range(num_obstacles):
            x = np.random.randint(0, self.MATRIX_DIV_W)
            y = np.random.randint(0, self.MATRIX_DflattenIV_H)
            self.matrix[x, y] = OCCUPIED

    def publish_static_transform(self):
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "map"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(transform)

    def timer_callback(self):
        # when we do this, we need to look at the frame id "base_footprint" so the matrix moves with use
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id = "map"
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.info.resolution = self.MATRIX_RES
        msg.info.width = self.MATRIX_DIV_W
        msg.info.height = self.MATRIX_DIV_H
        # set the matrix so that we are on the middle of it
        msg.info.origin.position.x = -self.MATRIX_WIDTH_m / 2
        msg.info.origin.position.y = -self.MATRIX_HEIGHT_m / 2
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0


        aux_matrix = self.matrix.flatten()
        msg.data = aux_matrix.tolist()
        self.publisher_.publish(msg)
        print("Published OccupancyGrid", "frame_id:", msg.header.frame_id, "stamp:", msg.header.stamp, "topic", self.publisher_.topic_name)


def main(args=None):
    print("main function")
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    os.system("ros2 topic list")
    main()  