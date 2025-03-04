import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import os
import math
import numpy as np
import matplotlib.pyplot as plt
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import Header



UN_INIT_VAL = -1
OCCUPIED = 1
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
        self.publisher_ = self.create_publisher(OccupancyGrid, "map_2", 10)
        self.timer = self.create_timer(2, self.timer_callback)






        # self.subscription = self.create_subscription(
        #     LaserScan,
        #     "/scan",
        #     self.listener_callback,
        #     10 # queue
        # )
        # print("scan subscribed")
        # self.subscription_odom = self.create_subscription(
        #     LaserScan,
        #     "/odom",
        #     self.lidar_data_2d_matrix_handler,
        #     10 # queue
        # )
        # print("odom subscribed")



        # self.subscription
        # self.subscription_odom
    def create_empty_matrix(self, rows, cols):
        # return np.full((rows, cols), UN_INIT_VAL)
        return np.full((rows, cols), FREE)


    def timer_callback(self):
        # publish the data to the topic "/OccupancyGrid"
        msg = OccupancyGrid()
        msg.header = Header()
        msg.header.frame_id           = "base_link"
        msg.header.stamp               = self.get_clock().now().to_msg()
        msg.info.resolution           = 0.1
        msg.info.width                = self.MATRIX_DIV_W
        msg.info.height               = self.MATRIX_DIV_H
        msg.info.origin.position.x    = 0.0
        msg.info.origin.position.y    = 0.0
        msg.info.origin.position.z    = 0.0
        msg.info.origin.orientation.x = 0.0
        msg.info.origin.orientation.y = 0.0
        msg.info.origin.orientation.z = 0.0
        msg.info.origin.orientation.w = 1.0
        # print("matrix_shape: ", self.matrix.shape)
        print(self.MATRIX_DIV_W, self.MATRIX_DIV_H)
        aux_matrix = np.full((self.MATRIX_DIV_W, self.MATRIX_DIV_H), 0)
        # convert to an integer matrix
        aux_matrix = aux_matrix.astype(int)
        # flatten the matrix
        aux_matrix = aux_matrix.flatten()

        msg.data = aux_matrix.tolist()
        # print("msg.data: ", msg.data)
        self.publisher_.publish(msg)
        print("Published OccupancyGrid", "frame_id: ", msg.header.frame_id, "stamp: ", msg.header.stamp, "topic", self.publisher_.topic_name)
        
        
        
    def listener_callback(self, msg):
        # self.get_logger().info("Received LiDAR data")
        # print all the data from msg
        msg.ranges = list(msg.ranges)
        # self.lidar_data_handler(msg
        # self.lidar_data_2d_matrix_handler(msg)
    def lidar_data_handler(self, data):
        # 'SLOT_TYPES', '__class__', '__delattr__', '__dir__', '__doc__',
        # '__eq__', '__format__', '__ge__', '__getattribute__',
        # '__gt__', '__hash__', '__init__', '__init_subclass__',
        # '__le__', '__lt__', '__module__', '__ne__', '__new__',
        # '__reduce__', '__reduce_ex__', '__repr__', '__setattr__',
        # '__sizeof__', '__slots__', '__str__', '__subclasshook__',
        # '_angle_increment', '_angle_max', '_angle_min', 
        # '_fields_and_field_types', '_header', '_intensities',
        # '_range_max', '_range_min', '_ranges', '_scan_time',
        # '_time_increment', 'angle_increment', 'angle_max', 'angle_min',
        # 'get_fields_and_field_types', 'header', 'intensities', 'range_max',
        # 'range_min', 'ranges', 'scan_time', 'time_increment'


        r = data.ranges
        print("Number of data points: ", len(r))
        print("Max distance: ", max(r))
        print("Min distance: ", min(r))
        print("Angle increment: ", data.angle_increment)
        print("Angle max: ", data.angle_max)
        print("Angle min: ", data.angle_min)
        # iterate over the data
        x = []
        y = []
        for i in range(len(r)):
            # angle is in radians
            angle = data.angle_min + i * data.angle_increment
            # print the angle and the distance
            print("i", i, "Angle: ", angle, " Distance: ", r[i])
            x.append(r[i] * math.cos(angle))
            y.append(r[i] * math.sin(angle))
        # plot the data
        plt.scatter(x, y)
        # plt.show()
        # display for 1 seconds and then continue
        plt.pause(0.5)
        # plt.close()
        # remove the trace and continue
        plt.clf()
    def lidar_data_2d_matrix_handler(self, data):

        # print all theparameters in the data
        print(dir(data))
        exit()  



        r = data.ranges



        print("Number of data points: ", len(r))
        # now we need to populate the matrix
        # iterate over the data
        x = []
        y = []
        for i in range(len(r)):
            # angle is in radians
            angle = data.angle_min + i * data.angle_increment
            # print the angle and the distance
            # print("i", i, "Angle: ", angle, " Distance: ", r[i])
            x.append(r[i] * math.cos(angle))
            y.append(r[i] * math.sin(angle))

        # now that we know the x and y coordinates, we can populate the matrix
        # iterate over the x and y coordinates
            for i in range(len(x)):
                # get the x and y coordinates
                x_coord = x[i]
                y_coord = y[i]

                # print("x_coord: ", x_coord, " y_coord: ", y_coord)
                if(math.isnan(x_coord) or math.isnan(y_coord)):
                    continue
                if(math.isinf(x_coord) or math.isinf(y_coord)):
                    continue
                

                angle   = data.angle_min + i * data.angle_increment
                # convert the x and y coordinates to matrix coordinates
                x_matrix = int((x_coord + self.MATRIX_WIDTH_m / 2) / self.MATRIX_WIDTH_m * self.MATRIX_DIV_W)
                y_matrix = int((y_coord + self.MATRIX_HEIGHT_m / 2) / self.MATRIX_HEIGHT_m * self.MATRIX_DIV_H)

                self.matrix[x_matrix][y_matrix] = OCCUPIED



        # plot the matrix
        plt.imshow(self.matrix, cmap="hot", interpolation="nearest")
        plt.pause(0.5)
        plt.clf()

        


        
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
