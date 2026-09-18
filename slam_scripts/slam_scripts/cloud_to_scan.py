#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, LaserScan
import sensor_msgs_py.point_cloud2 as pc2

class PointCloudToLaserScanPy(Node):
    def __init__(self):
        super().__init__('cloud_to_scan_py')
        self.sub = self.create_subscription(
            PointCloud2,
            '/velodyne_points/points',
            self.cloud_callback,
            10
        )
        self.pub = self.create_publisher(LaserScan, '/scan', 10)
        self.get_logger().info("Cloud-to-LaserScan converter node is active.")

    def cloud_callback(self, cloud_msg):
        angle_min = -math.pi
        angle_max = math.pi
        angle_inc = math.radians(1.0)  # 360 rays (1.0 deg resolution)
        num_readings = int(round((angle_max - angle_min) / angle_inc))

        ranges = [float('inf')] * num_readings
        intensities = [100.0] * num_readings  # Dummy intensity to satisfy RViz2 Color Transformer

        for p in pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True):
            x, y, z = p[0], p[1], p[2]

            # Filter ground reflections (-0.08m) and ceiling hits (+1.2m)
            if z < -0.08 or z > 1.2:
                continue

            r = math.hypot(x, y)
            if r < 0.25 or r > 25.0:
                continue

            angle = math.atan2(y, x)
            idx = int(round((angle - angle_min) / angle_inc))
            if 0 <= idx < num_readings:
                if r < ranges[idx]:
                    ranges[idx] = r

        scan = LaserScan()
        scan.header = cloud_msg.header
        scan.angle_min = angle_min
        scan.angle_max = angle_max
        scan.angle_increment = angle_inc
        scan.time_increment = 0.0
        scan.scan_time = 0.1
        scan.range_min = 0.25
        scan.range_max = 25.0
        scan.ranges = ranges
        scan.intensities = intensities

        self.pub.publish(scan)

def main():
    rclpy.init()
    node = PointCloudToLaserScanPy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
