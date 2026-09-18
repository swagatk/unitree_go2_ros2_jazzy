#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

class OdomToFootprint(Node):
    def __init__(self):
        super().__init__('odom_to_footprint_broadcaster')
        self.br = TransformBroadcaster(self)

        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.handle_odom,
            qos
        )
        self.get_logger().info("Dynamic TF Broadcaster: odom -> base_footprint initialized.")

    def handle_odom(self, msg):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_footprint'
        
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = 0.0
        t.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = OdomToFootprint()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
