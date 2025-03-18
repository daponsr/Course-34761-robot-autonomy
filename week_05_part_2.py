import rclpy
from myoccupancygrid import MyOccupancyGrid

def main():
    rclpy.init()
    node = MyOccupancyGrid("update_map")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main() 
