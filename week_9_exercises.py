#!/usr/bin/env python3
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
# import the Map
# from nav_msgs.msg import Map
# import the MarkerArray
from visualization_msgs.msg import MarkerArray, Marker
# from nav_msgs.msg import Map
import random

from typing import Tuple

from rclpy.time import Time

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion



class Lecture0(Node):
    def __init__(self):
        self.node = rclpy.create_node("update_map")
        # subscribe to the topic /map
        self.subscription = self.node.create_subscription(OccupancyGrid, "/map", self.callback_map, 10)
        self.subscription
        # create a number of random nodes in the free area
        self.amount_of_nodes = 10
        self.nodes_init = False
        self.random_nodes = []

        # publisher to "prm_markers"
        self.publisher = self.node.create_publisher(MarkerArray, "prm_markers", 10)
        self.publisher


    def callback_map(self, msg):
        print("map received")
        if(not self.nodes_init):
            self.nodes_init = True

            # printe what I get in the map
            print("map data")
            # print a list of all the elements in the "msg" object
            print(dir(msg))
            print("info", msg.info)
            # prin the shape of the map
            print("shape", np.array(msg.data).shape)
            # convert the data to a matrix
            print("data", np.array(msg.data))

            self.random_nodes = self.generate_random_nodes(msg)
            # reshape the data to a 3D matrix with the shape (height, width, 1)
            # publish the nodes
        self.publish_nodes()

    def publish_nodes(self):
        print("publishing nodes")
        # create the markers
        markers = MarkerArray()
        for i in range(len(self.random_nodes)):
            marker = Marker()
            marker.header.frame_id = "map"
            marker.header.stamp = self.node.get_clock().now().to_msg()
            marker.id = i
            marker.type = marker.SPHERE
            marker.action = marker.ADD
            marker.pose.position.x = self.random_nodes[i][0]
            marker.pose.position.y = self.random_nodes[i][1]
            print("node", marker.pose.position.x, marker.pose.position.y)
            marker.pose.position.z = 0.5
            marker.scale.x = 0.2
            marker.scale.y = 0.2
            marker.scale.z = 0.2
            marker.color.a = 1.0
            marker.color.r = 0.0
            marker.color.g = 1.0
            marker.color.b = 0.0
            markers.markers.append(marker)

        # publish the markers
        self.publisher.publish(markers)

    def generate_random_nodes(self, msg):
        print("generating random nodes")
        # create a list of random nodes
        random_nodes = []
        for i in range(self.amount_of_nodes):
            # generate a random node
            random_node = self.generate_random_node(msg)
            # convert the "node" to a x, y

            coordinate_x = random_node[0]*msg.info.resolution + msg.info.origin.position.x
            coordinate_y = random_node[1]*msg.info.resolution + msg.info.origin.position.y

            random_nodes.append((coordinate_x, coordinate_y))
        return random_nodes
    
    def generate_random_node(self, msg):
        # generate a random node
        while True:
            # generate a random node
            random_node = (random.uniform(0, msg.info.width), random.uniform(0, msg.info.height))
            # check if the node is in the free area
            if(self.is_free(random_node, msg)):
                return random_node
    def is_free(self, node, msg):
        # check if the node is in the array
        # the matrix is repseted as 1D array
        # the index of the matrix is calculated as "y * width + x"
        index = int(node[1] * msg.info.width + node[0])
        # check if the node is in the free area
        if(index < 0 or index >= len(msg.data)):
            return False

        if(msg.data[index] == 0):
            return True
        return False






def main():
    print("week 9 exercises")
    rclpy.init()
    node = Lecture0()
    rclpy.spin(node.node)
    rclpy.shutdown()



if __name__ == "__main__":
    main()