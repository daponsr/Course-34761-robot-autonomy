import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from myoccupancygrid import MyOccupancyGrid
import time

class OdometryMapping(Node):
    def __init__(self):
        print("Odometry Mapping Node Started")
        super().__init__('odometry_mapping')
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.current_pose = Pose()
        self.occupancy_grid = MyOccupancyGrid("update_map_with_counter")
        # set that we need to use the update with counters
        rclpy.spin(self.occupancy_grid)
        self.occupancy_grid.destroy_node()
        # done with init
        print("init done")



    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose
        # self.occupancy_grid.update_map_2(self.current_pose)
        # self.occupancy_grid.update_map_with_counters(self.current_pose)
        time.sleep(1)

def main():
    rclpy.init()
    node = OdometryMapping()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()