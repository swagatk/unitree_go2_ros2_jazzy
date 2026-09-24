#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import tf2_ros


class ExecutedPathPublisher(Node):
    """
    Subscribes to TF transformations (map -> base_footprint)
    and publishes an accumulated nav_msgs/msg/Path representing
    the true localized trajectory executed by the quadruped.
    """

    def __init__(self):
        super().__init__('executed_path_publisher')

        self.path_pub = self.create_publisher(Path, '/executed_path', 10)
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.path_msg = Path()
        self.path_msg.header.frame_id = 'map'

        self.last_x = None
        self.last_y = None
        self.min_distance_threshold = 0.03  # Add point every 3 cm of movement

        # Sample trajectory at 10 Hz
        self.timer = self.create_timer(0.1, self.record_pose)
        self.get_logger().info("Executed path publisher active on topic: /executed_path")

    def record_pose(self):
        try:
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint',
                rclpy.time.Time()
            )
        except (
            tf2_ros.LookupException,
            tf2_ros.ConnectivityException,
            tf2_ros.ExtrapolationException
        ):
            return

        curr_x = t.transform.translation.x
        curr_y = t.transform.translation.y

        if self.last_x is not None and self.last_y is not None:
            dist = math.hypot(curr_x - self.last_x, curr_y - self.last_y)
            if dist < self.min_distance_threshold:
                return

        self.last_x = curr_x
        self.last_y = curr_y

        pose = PoseStamped()
        pose.header = t.header
        pose.pose.position.x = curr_x
        pose.pose.position.y = curr_y
        pose.pose.position.z = t.transform.translation.z
        pose.pose.orientation = t.transform.rotation

        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(pose)

        if len(self.path_msg.poses) > 5000:
            self.path_msg.poses.pop(0)

        self.path_pub.publish(self.path_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ExecutedPathPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
