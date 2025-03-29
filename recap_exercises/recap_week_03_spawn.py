#!/usr/bin/env python3

import os
import sys
import time
import rclpy
import math
from rclpy.node import Node
from geometry_msgs.msg import Twist

reca

class TurtleController():
    def __init__(self, node, list_of_turtles):
        self.node = node
        self.list_of_turtles = list_of_turtles
        self.publisher = {}
        for turtle in self.list_of_turtles:
            self.publisher[turtle] = self.node.create_publisher(Twist, f'{turtle}/cmd_vel', 10)
        self.timer = self.node.create_timer(0.1, self.timer_callback)
        self.next_time_change = time.time() + 5
        self.val_direction = 1

    def timer_callback(self):
        print("Timer callback")
        msg = Twist()

        if(time.time() > self.next_time_change):
            self.next_time_change = time.time() + 5
            self.val_direction = -self.val_direction
        msg.linear.x = 1.0*self.val_direction
        msg.angular.z = 1.0

        for turtle in self.publisher:
            self.publisher[turtle].publish(msg)

    # def convert_from_leter_to_speed_pattern(self, leter):
        # we want to move the turtle s



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
            dict_kill = {"name": turtle_name}
            cmd = f"ros2 service call /kill turtlesim/srv/Kill \"{dict_kill}\""
            os.system(cmd)

        cmd = f"ros2 service call /spawn turtlesim/srv/Spawn '{dict_args}'"
        os.system(cmd)



    rclpy.init(args=argv)
    node = rclpy.create_node('turtle_controller')
    turtle_controller = TurtleController(node, turtle_names)
    rclpy.spin(node)
    




if __name__ == "__main__":
    main(sys.argv[1:])
