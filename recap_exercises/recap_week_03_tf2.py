#!/usr/bin/env python3

import os
import sys
import time
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from tf2_ros import LookupException

class TurtleController():
    def __init__(self, node, list_of_turtles):
        self.node = node
        self.turtle_to_follow = "turtle1"
        self.list_of_turtles = list_of_turtles # this are the ones that will be following
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self.node)
        self._cmd_vel = Twist()
        self.publisher = {}
        for turtle in self.list_of_turtles:
            self.publisher[turtle] = self.node.create_publisher(Twist, f"/{turtle}/cmd_vel", 10)


        self.timer = self.node.create_timer(1.0, self.timer_callback)

    def timer_callback(self):
        print("Timer callback")
        # now we want to use it so the turtle get the transform relative to "turtle1" and actually move there
        for turtle in self.list_of_turtles:
            try:
                print("from turtle: ", turtle)
                trans = self._tf_buffer.lookup_transform(turtle, self.turtle_to_follow, rclpy.time.Time())
                self._cmd_vel.linear.x = math.sqrt(trans.transform.translation.x ** 2 + trans.transform.translation.y ** 2)
                self._cmd_vel.angular.z = math.atan2(trans.transform.translation.y , trans.transform.translation.x)
                self.publisher[turtle].publish(self._cmd_vel)
            except LookupException as e:
                print(f"Error: {e}")
                continue


AMOUNT_OF_SCREENS = 1
def main(argv):
    amount_of_turtle = int(argv[0])

    turtle_names = []

    for i in range(amount_of_turtle):
        turtle_name = "my_turtle" + str(i)
        turtle_names.append(turtle_name)
        x_coordinate = 5 + 2*math.cos(2*math.pi*i/amount_of_turtle)
        y_coordinate = 5 + 2*math.sin(2*math.pi*i/amount_of_turtle)
        dict_args = {"x": x_coordinate, "y": y_coordinate, "name": turtle_name}
        print(dict_args)

                # check if the turtle already exists
        os.system("ros2 service list | grep set_pen > output.txt")
        f = open("output.txt", "r")
        lines = f.read()
        if(turtle_name in lines):
            print(f"Turtle {turtle_name} already exists")
            # kill the turtle
            continue

            dict_kill = {"name": turtle_name}
            cmd = f"ros2 service call /kill turtlesim/srv/Kill \"{dict_kill}\""
            os.system(cmd)

        cmd = f"ros2 service call /spawn turtlesim/srv/Spawn '{dict_args}'"
        os.system(cmd)

        # # call the script the
        cmd = f"./recap_exercises/recap_week_03_tf2_talker.py {turtle_name} &"
        os.system(cmd)


    rclpy.init(args=argv)
    node = rclpy.create_node('turtle_controller')
    turtle_controller = TurtleController(node, turtle_names)
    rclpy.spin(node)

if __name__ == "__main__":
    main(sys.argv[1:])

