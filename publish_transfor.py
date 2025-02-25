import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros
import math
import time

class TransformPublisher(Node):
    def __init__(self):
        super().__init__('transform_publisher')
        self.br = tf2_ros.TransformBroadcaster(self)
        self.odom_publisher = self.create_publisher(Odometry, 'my_odom', 10)
        self.timer = self.create_timer(1, self.timer_callback)
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.odom_sub = self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        print("init --> TransformPublisher")

    def odom_callback(self, msg):
        print("odom_callback")
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        orientation = msg.pose.pose.orientation
        _, _, self.yaw = self.quaternion_to_euler(orientation.x, orientation.y, orientation.z, orientation.w)
        print(f"Received odom data: x={self.x:2.2f}, y={self.y:2.2f}, yaw={self.yaw:2.2f}")

    def quaternion_to_euler(self, x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll = math.atan2(t0, t1)

        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch = math.asin(t2)

        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw = math.atan2(t3, t4)

        return roll, pitch, yaw

    def timer_callback(self):

        q = self.euler_to_quaternion(0, 0, self.yaw)


        # Publish the transform
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'my_odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.br.sendTransform(t)
        print(f"Published TransformStamped: x={self.x:2.2f}, y={self.y:2.2f}, yaw={self.yaw:2.2f}")

        # Publish the odometry message
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'my_odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = q[0]
        odom_msg.pose.pose.orientation.y = q[1]
        odom_msg.pose.pose.orientation.z = q[2]
        odom_msg.pose.pose.orientation.w = q[3]
        self.odom_publisher.publish(odom_msg)

        print(f"Published Odometry: x={q[0]:2.2f}, y={q[1]:2.2f}, yaw={q[3]:2.2f}")

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = TransformPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    print("main")
    main()