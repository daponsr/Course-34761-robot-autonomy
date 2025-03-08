


# this is the one use 
# import rclpy
# from rclpy.executors import ExternalShutdownException, MultiThreadedExecutor
# from turtlebot3occupancygrid import Turtlebot3OccupancyGrid

# def main(*args, **kwargs):
#     rclpy.init(*args, **kwargs)
#     node = Turtlebot3OccupancyGrid(node_name="turtlebot3_occupancy_grid")
#     # rclpy.spin(node)
#     # node.destroy_node()
#     # rclpy.try_shutdown()
#     executor = MultiThreadedExecutor()
#     try:
#         rclpy.spin(node=node, executor=executor)
#     except (KeyboardInterrupt, ExternalShutdownException):
#         pass
#     finally:
#         rclpy.try_shutdown()

import rclpy
from myoccupancygrid import MyOccupancyGrid

def main():
    rclpy.init()
    node = MyOccupancyGrid()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main() 
