import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import os
import math
import numpy as np
import matplotlib.pyplot as plt

class LidarSubscriber(Node):
    def __init__(self):
        super().__init__("lidar_subscriber")
        self.subscription = self.create_subscription(
            LaserScan,
            "/scan",
            self.listener_callback,
            10 # queue
        )
        self.subscription
    def listener_callback(self, msg):
        self.get_logger().info("Received LiDAR data")
        # print all the data from msg
        msg.ranges = list(msg.ranges)
        self.lidar_data_handler(msg)
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
        


        




def main(args=None):
    print("main function")
    rclpy.init(args=args)
    lidar_subscriber = LidarSubscriber()
    rclpy.spin(lidar_subscriber)
    lidar_subscriber.destroy_node()
    rclpy.shutdown()

# for the rviz2 visualisation --> select base_link in order to getthe lidar data
# then add the "Laser" and it will be displayed

# run this before the script
# source install/setup.bash
# export ROS_DOMAIN_ID=11
# export TURTLEBOT3_MODEL=burger
# export GAZEBO_MODEL_PATH=$GAZEBO_MODL_PATH:$(ros2 pkg prefix my_turtlebot)/share/my_turtlebot/models
# export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(ros2 pkg prefix turtlebot3_gazebo)/share/turtlebot3_gazebo/models




if __name__ == "__main__":
    os.system("ros2 topic list")
    main()
