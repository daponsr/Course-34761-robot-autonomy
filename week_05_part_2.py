# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import LaserScan
# from geometry_msgs.msg import PoseStamped
# from nav_msgs.msg import Odometry
# import os
# import math
# import numpy as np
# import matplotlib.pyplot as plt
# from geometry_msgs.msg import TransformStamped
# from geometry_msgs.msg import PointStamped
# from tf2_geometry_msgs import do_transform_point
# from nav_msgs.msg import OccupancyGrid
# from std_msgs.msg import Header
# import tf2_ros

# UN_INIT_VAL = -1
# OCCUPIED = 100
# FREE = 0

# class LidarSubscriber(Node):
#     def __init__(self):
#         super().__init__("lidar_subscriber")
#         # create the empty matrix
#         self.MATRIX_RES = 0.01
#         self.MATRIX_WIDTH_m = 6
#         self.MATRIX_HEIGHT_m = 6
#         self.MATRIX_DIV_W = int(self.MATRIX_WIDTH_m / self.MATRIX_RES)
#         self.MATRIX_DIV_H = int(self.MATRIX_HEIGHT_m / self.MATRIX_RES)
#         self.matrix = self.create_empty_matrix(self.MATRIX_DIV_W, self.MATRIX_DIV_H)
#         self.populate_random_obstacles()

#         self.publisher_ = self.create_publisher(OccupancyGrid, "/map_2", 10)
#         self.timer = self.create_timer(2, self.timer_callback)
#         # subscribe to the lidar data
#         self.create_subscription(LaserScan, "/scan", self.lidar_callback, 10)

#         # Initialize the static transform broadcaster
#         self.static_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
#         self.publish_static_transform()
#         # init the position to unknown
#         self.x = 0
#         self.y = 0
#         self.yaw = 0



#         self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)

#     def odom_callback(self, msg):
#         self.x = msg.pose.pose.position.x
#         self.y = msg.pose.pose.position.y
#         orientation = msg.pose.pose.orientation
#         _, _, self.yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
#         # print(f"Received odom data: x={self.x:2.2f}, y={self.y:2.2f}, yaw={self.yaw:2.2f}")

#     def quaternion_to_euler(self, x, y, z, w):
#         t0 = +2.0 * (w * x + y * z)
#         t1 = +1.0 - 2.0 * (x * x + y * y)
#         roll = math.atan2(t0, t1)

#         t2 = +2.0 * (w * y - z * x)
#         t2 = +1.0 if t2 > +1.0 else t2
#         t2 = -1.0 if t2 < -1.0 else t2
#         pitch = math.asin(t2)

#         t3 = +2.0 * (w * z + x * y)
#         t4 = +1.0 - 2.0 * (y * y + z * z)
#         yaw = math.atan2(t3, t4)

#         return roll, pitch, yaw

#     def create_empty_matrix(self, rows, cols):
#         return np.full((rows, cols), FREE)

#     def populate_random_obstacles(self):
#         num_obstacles = int(0.1 * self.MATRIX_DIV_W * self.MATRIX_DIV_H)  # 10% of the cells
#         for _ in range(num_obstacles):
#             x = np.random.randint(0, self.MATRIX_DIV_W)
#             y = np.random.randint(0, self.MATRIX_DIV_H)
#             self.matrix[x, y] = OCCUPIED

#     def publish_static_transform(self):
#         transform = TransformStamped()
#         transform.header.stamp = self.get_clock().now().to_msg()
#         transform.header.frame_id = "map"
#         transform.child_frame_id = "base_link"
#         transform.transform.translation.x = 0.0
#         transform.transform.translation.y = 0.0
#         transform.transform.translation.z = 0.0
#         transform.transform.rotation.x = 0.0
#         transform.transform.rotation.y = 0.0
#         transform.transform.rotation.z = 0.0
#         transform.transform.rotation.w = 1.0
#         self.static_broadcaster.sendTransform(transform)

#     def timer_callback(self):
#         # when we do this, we need to look at the frame id "base_footprint" so the matrix moves with use
#         msg = OccupancyGrid()
#         msg.header = Header()
#         msg.header.frame_id = "map"
#         msg.header.stamp = self.get_clock().now().to_msg()
#         msg.info.resolution = self.MATRIX_RES
#         msg.info.width = self.MATRIX_DIV_W
#         msg.info.height = self.MATRIX_DIV_H
#         # set the matrix so that we are on the middle of it
#         msg.info.origin.position.x = -self.MATRIX_WIDTH_m / 2
#         msg.info.origin.position.y = -self.MATRIX_HEIGHT_m / 2
#         msg.info.origin.position.z = 0.0
#         msg.info.origin.orientation.x = 0.0
#         msg.info.origin.orientation.y = 0.0
#         msg.info.origin.orientation.z = 0.0
#         msg.info.origin.orientation.w = 1.0
#         # msg.info.origin.orientation.x = 0.0
#         # msg.info.origin.orientation.y = 0.0
#         # msg.info.origin.orientation.z = -math.sin(self.yaw / 2)
#         # msg.info.origin.orientation.w = -math.cos(self.yaw / 2)




#         aux_matrix = self.matrix.flatten()
#         msg.data = aux_matrix.tolist()
#         self.publisher_.publish(msg)
#         print("Published OccupancyGrid", "frame_id:", msg.header.frame_id, "stamp:", msg.header.stamp, "topic", self.publisher_.topic_name)
#         # clear the matrix
#         self.matrix = self.create_empty_matrix(self.MATRIX_DIV_W, self.MATRIX_DIV_H)

#     def lidar_callback(self, msg):
#         # mark the cells as occupied
#         # need to take into account that we are in the middle of the matrix
#         # we need to take into account the angle of the lidar


#         data = np.array(msg.ranges)
#         all_xs = data * np.sin(msg.angle_min + np.arange(len(data)) * msg.angle_increment)
#         all_ys = data * np.cos(msg.angle_min + np.arange(len(data)) * msg.angle_increment)
#         # now we need to convert the xs and ys to the matrix in order to map them
#         for x, y in zip(all_xs, all_ys):
#             # check if the point is within the matrix
#             if math.isinf(x) or math.isinf(y):
#                 continue
#             if math.isnan(x) or math.isnan(y):
#                 continue
#             if x < -self.MATRIX_WIDTH_m / 2 or x > self.MATRIX_WIDTH_m / 2:
#                 continue
#             # convert to matrix coordinates
#             x_matrix = int((x + self.MATRIX_WIDTH_m / 2) / self.MATRIX_RES)
#             y_matrix = int((y + self.MATRIX_HEIGHT_m / 2) / self.MATRIX_RES)
#             if x_matrix >= 0 and x_matrix < self.MATRIX_DIV_W and y_matrix >= 0 and y_matrix < self.MATRIX_DIV_H:
#                 self.matrix[x_matrix, y_matrix] = OCCUPIED

# def main(args=None):
#     print("main function")
#     rclpy.init(args=args)
#     lidar_subscriber = LidarSubscriber()
#     rclpy.spin(lidar_subscriber)
#     lidar_subscriber.destroy_node()
#     rclpy.shutdown()

# if __name__ == "__main__":
#     os.system("ros2 topic list")
#     main() 
# 
 
import rclpy
from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
from turtlebot3occupancygrid import Turtlebot3OccupancyGrid

def main(*args, **kwargs):
    rclpy.init(*args, **kwargs)
    node = Turtlebot3OccupancyGrid(node_name="turtlebot3_occupancy_grid")
    rclpy.spin(node)
    # executor = MultiThreadedExecutor()
    # try:
        # rclpy.spin(node=node, executor=executor)
    # except (KeyboardInterrupt, ExternalShutdownException):
    #     pass
    # finally:
    #     rclpy.try_shutdown()


if __name__ == "__main__":
    main() 
