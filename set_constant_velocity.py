giimport rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
import os
import sys
import math
import numpy as np
import matplotlib.pyplot as plt


# topic to send velocity commands to the robot
# topic is "/cmd_vel"
class Turtlebot3Publisher(Node):
    def __init__(self, linear, angular):
        super().__init__("turtlebot3_publisher")
        self.publisher_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.linear  = linear
        self.angular = angular
        self.timer = self.create_timer(10, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = Twist()
        msg.linear.x  = self.linear
        msg.angular.z = self.angular
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.linear.x}, {msg.angular.z}")
        self.i += 1

        
def main(linead, angle):
    rclpy.init()
    node = Turtlebot3Publisher(linead, angle)
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    os.system("ros2 topic list")
    # get the linear and angular from the arguments
    linear  = sys.argv[1]
    angular = sys.argv[2]
    # parse to floats
    linear  = float(linear)
    angular = float(angular)
    main(linear, angular)
