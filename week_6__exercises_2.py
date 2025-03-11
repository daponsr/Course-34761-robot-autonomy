import os
import rclpy
from nav2_msgs.msg import ParticleCloud  # Ensure correct import
from sensor_msgs.msg import LaserScan
import numpy as np
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ParticleCloudSubscriber(Node):
    def __init__(self):
        super().__init__('listener_cloud_sub')
        
        # Define QoS settings
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.particle_cloud_sub = self.create_subscription(
            ParticleCloud, 
            "/particle_cloud", 
            self.particle_cloud_callback, 
            qos_profile
        )
        
        
        print("Subscribed to /particle_cloud")
    
    def particle_cloud_callback(self, msg):
        print("amount of particles", len(msg.particles))
    

def main():
    rclpy.init()
    node = ParticleCloudSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    os.system("ros2 topic list")
    main()