import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import os



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