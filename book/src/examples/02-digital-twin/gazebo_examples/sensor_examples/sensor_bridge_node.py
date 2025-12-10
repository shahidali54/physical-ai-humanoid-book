#!/usr/bin/env python3

"""
Sensor Bridge Node for Digital Twin Examples

This node demonstrates how to bridge sensor data between simulation and visualization.
It subscribes to simulated sensor topics and republishes them for visualization or
further processing.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, LaserScan, Imu
from std_msgs.msg import Header
import numpy as np
import math


class SensorBridgeNode(Node):
    """
    A node that subscribes to various sensor topics and processes the data.
    This simulates the kind of bridge that would connect simulation sensors
    to visualization or processing systems in a digital twin environment.
    """

    def __init__(self):
        super().__init__('sensor_bridge_node')

        # Create subscribers for different sensor types
        self.camera_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.camera_callback,
            10
        )

        self.lidar_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Create publishers for processed data
        self.processed_camera_pub = self.create_publisher(
            Image,
            '/camera/processed',
            10
        )

        self.get_logger().info('Sensor Bridge Node initialized')

    def camera_callback(self, msg):
        """Process camera data from simulation"""
        self.get_logger().info(f'Camera data received: {msg.width}x{msg.height} image')
        # In a real implementation, this would process the image data
        # For now, we just log the information

        # Re-publish processed data (simulated)
        processed_msg = msg
        processed_msg.header.stamp = self.get_clock().now().to_msg()
        self.processed_camera_pub.publish(processed_msg)

    def lidar_callback(self, msg):
        """Process LIDAR data from simulation"""
        range_count = len(msg.ranges)
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]

        self.get_logger().info(
            f'LIDAR data received: {range_count} ranges, '
            f'{len(valid_ranges)} valid measurements'
        )

        # Example: Find closest obstacle
        if valid_ranges:
            min_range = min(valid_ranges)
            self.get_logger().info(f'Closest obstacle: {min_range:.2f}m')

    def imu_callback(self, msg):
        """Process IMU data from simulation"""
        linear_accel = math.sqrt(
            msg.linear_acceleration.x**2 +
            msg.linear_acceleration.y**2 +
            msg.linear_acceleration.z**2
        )

        angular_vel = math.sqrt(
            msg.angular_velocity.x**2 +
            msg.angular_velocity.y**2 +
            msg.angular_velocity.z**2
        )

        self.get_logger().info(
            f'IMU data - Accel: {linear_accel:.2f} m/s², '
            f'Angular Vel: {angular_vel:.2f} rad/s'
        )


def main(args=None):
    rclpy.init(args=args)

    sensor_bridge_node = SensorBridgeNode()

    try:
        rclpy.spin(sensor_bridge_node)
    except KeyboardInterrupt:
        pass
    finally:
        sensor_bridge_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()