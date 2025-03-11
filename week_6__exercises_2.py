import os
import rclpy
from nav2_msgs.msg import ParticleCloud  # Ensure correct import
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

class ParticleCloudSubscriber(Node):
    def __init__(self):
        super().__init__('listener_cloud_sub')
        
        self.avg_x = 0.0
        self.avg_y = 0.0

        self.compute_x = 0.0
        self.compute_y = 0.0

        self.pose_x = 0.0
        self.pose_y = 0.0

        self.particle_count = 0



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
        
        self.amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped, 
            "/amcl_pose", 
            self.amcl_pose_callback, 
            qos_profile
        )

        # add a time callback
        self.create_timer(1.0, self.timer_callback)
        
        print("Subscribed to /particle_cloud and /amcl_pose")
    
    def timer_callback(self):
        print("Timer callback")
        print(f"Amount of samples received:             {self.particle_count:2.5f}")
        print(f"Average position:                  x = {self.avg_x:2.5f}, y = {self.avg_y:2.5f}")
        print(f"Computed position:                 x = {self.compute_x:2.5f}, y = {self.compute_y:2.5f}")
        print(f"AMCL estimated position:           x = {self.pose_x:2.5f}, y = {self.pose_y:2.5f}")
        # diff between average  Computed and AMCL
        print(f"Diff between average and Computed: x = {self.avg_x - self.compute_x:2.5f}, y = {self.avg_y - self.compute_y:2.5f}")

    def particle_cloud_callback(self, msg):
        # print("amount of particles", len(msg.particles))
        self.particle_count = len(msg.particles)
        positions = [(particle.pose.position.x, particle.pose.position.y) for particle in msg.particles]
        
        # Compute the weighted average position
        x_sum = sum(p[0] for p in positions)
        y_sum = sum(p[1] for p in positions)
        num_particles = len(positions)
        
        if num_particles > 0:
            x_avg = x_sum / num_particles
            y_avg = y_sum / num_particles
        else:
            print("No particles received")
            return
        
        # Take the real x and y position that is closer to the average position
        min_x_dist = float('inf')
        min_y_dist = float('inf')
        real_x = float('inf')
        real_y = float('inf')

        for particle in msg.particles:
            x_dist = abs(particle.pose.position.x - x_avg)
            y_dist = abs(particle.pose.position.y - y_avg)
            if x_dist < min_x_dist:
                min_x_dist = x_dist
                real_x = particle.pose.position.x
            if y_dist < min_y_dist:
                min_y_dist = y_dist
                real_y = particle.pose.position.y
        
        self.avg_x = x_avg
        self.avg_y = y_avg

        self.compute_x = real_x
        self.compute_y = real_y
        
        # print(f"Estimated position:      x = {x_avg:2.5f}, y = {y_avg:2.5f}")
        # print(f"Real position:           x = {real_x:2.5f}, y = {real_y:2.5f}")
    
    def amcl_pose_callback(self, msg):
        # Extract the estimated position from the AMCL pose
        self.pose_x = msg.pose.pose.position.x
        self.pose_y = msg.pose.pose.position.y
        # print(f"AMCL estimated position: x = {estimated_x:2.5f}, y = {estimated_y:2.5f}")

def main():
    rclpy.init()
    node = ParticleCloudSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    os.system("ros2 topic list")
    main()