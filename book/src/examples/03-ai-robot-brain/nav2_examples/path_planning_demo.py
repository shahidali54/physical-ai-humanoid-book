#!/usr/bin/env python3
"""
Humanoid Path Planning Demo for Nav2

This script demonstrates path planning specifically adapted for humanoid robots,
considering unique kinematic constraints and balance requirements of bipedal locomotion.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import Header
import numpy as np
import math


class HumanoidPathPlannerNode(Node):
    """
    Humanoid Path Planner Node
    Demonstrates Nav2 path planning adapted for humanoid robots
    """

    def __init__(self):
        super().__init__('humanoid_path_planner')

        # Parameters for humanoid-specific planning
        self.declare_parameter('robot_step_length', 0.3)  # meters
        self.declare_parameter('robot_step_height', 0.15)  # meters
        self.declare_parameter('robot_width', 0.4)  # meters
        self.declare_parameter('robot_height', 1.0)  # meters
        self.declare_parameter('balance_margin', 0.1)  # safety margin

        self.step_length = self.get_parameter('robot_step_length').value
        self.step_height = self.get_parameter('robot_step_height').value
        self.robot_width = self.get_parameter('robot_width').value
        self.robot_height = self.get_parameter('robot_height').value
        self.balance_margin = self.get_parameter('balance_margin').value

        # Publishers
        self.path_pub = self.create_publisher(Path, '/humanoid_plan', 10)
        self.footstep_pub = self.create_publisher(MarkerArray, '/footsteps', 10)
        self.balance_pub = self.create_publisher(Marker, '/balance_zone', 10)

        # Subscribers
        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.get_logger().info("Humanoid Path Planner node initialized")

    def goal_callback(self, msg):
        """Process navigation goal and generate humanoid-aware path"""
        self.get_logger().info(f"Received goal: ({msg.pose.position.x}, {msg.pose.position.y})")

        # Generate path considering humanoid constraints
        path = self.generate_humanoid_path(msg.pose)

        # Publish the path
        self.path_pub.publish(path)

        # Generate footstep plan
        footsteps = self.generate_footsteps(path)

        # Publish footsteps visualization
        self.footstep_pub.publish(footsteps)

        # Publish balance zone visualization
        balance_zone = self.create_balance_zone(msg.pose)
        self.balance_pub.publish(balance_zone)

    def generate_humanoid_path(self, goal_pose):
        """Generate path considering humanoid kinematic constraints"""
        path = Path()
        path.header = Header()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = 'map'

        # Start position (simplified)
        start_x, start_y = 0.0, 0.0
        goal_x, goal_y = goal_pose.position.x, goal_pose.position.y

        # Calculate path points with humanoid-specific constraints
        # For demonstration, create a simple straight-line path with intermediate waypoints
        # that respect step length limits
        distance = math.sqrt((goal_x - start_x)**2 + (goal_y - start_y)**2)
        num_waypoints = max(2, int(distance / self.step_length))

        for i in range(num_waypoints + 1):
            t = i / num_waypoints if num_waypoints > 0 else 0
            x = start_x + t * (goal_x - start_x)
            y = start_y + t * (goal_y - start_y)

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.position.z = 0.0

            # Simple orientation toward goal
            yaw = math.atan2(goal_y - y, goal_x - x)
            pose.pose.orientation.z = math.sin(yaw / 2.0)
            pose.pose.orientation.w = math.cos(yaw / 2.0)

            path.poses.append(pose)

        self.get_logger().info(f"Generated path with {len(path.poses)} waypoints")
        return path

    def generate_footsteps(self, path):
        """Generate footstep plan for bipedal locomotion"""
        footsteps = MarkerArray()

        # Create markers for left and right foot steps
        for i, pose_stamped in enumerate(path.poses):
            if i % 2 == 0:  # Left foot
                foot_marker = self.create_foot_marker(
                    pose_stamped.pose.position,
                    i,
                    'left_foot',
                    [0, 0, 1, 0.8]  # Blue
                )
            else:  # Right foot
                foot_marker = self.create_foot_marker(
                    pose_stamped.pose.position,
                    i,
                    'right_foot',
                    [1, 0, 0, 0.8]  # Red
                )

            footsteps.markers.append(foot_marker)

        return footsteps

    def create_foot_marker(self, position, step_id, foot_name, color):
        """Create a marker for a footstep"""
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'footsteps'
        marker.id = step_id
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose.position = position
        marker.pose.position.z = 0.02  # Slightly above ground

        marker.scale.x = 0.15  # Foot length
        marker.scale.y = 0.1   # Foot width
        marker.scale.z = 0.01  # Foot height

        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        marker.lifetime.sec = 30  # 30 seconds

        return marker

    def create_balance_zone(self, goal_pose):
        """Create a visualization marker for the robot's balance zone"""
        marker = Marker()
        marker.header = Header()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = 'map'
        marker.ns = 'balance_zone'
        marker.id = 0
        marker.type = Marker.CYLINDER
        marker.action = Marker.ADD

        marker.pose = goal_pose
        marker.pose.position.z = 0.1  # Above ground

        # Balance zone radius based on robot dimensions and safety margin
        balance_radius = self.robot_width / 2 + self.balance_margin
        marker.scale.x = balance_radius * 2
        marker.scale.y = balance_radius * 2
        marker.scale.z = 0.02

        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0
        marker.color.a = 0.3  # Semi-transparent

        marker.lifetime.sec = 30

        return marker


def main(args=None):
    rclpy.init(args=args)

    planner_node = HumanoidPathPlannerNode()

    try:
        rclpy.spin(planner_node)
    except KeyboardInterrupt:
        pass
    finally:
        planner_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()