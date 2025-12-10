#!/usr/bin/env python3
"""
Isaac ROS VSLAM Pipeline Example

This script demonstrates a basic Visual SLAM pipeline using Isaac ROS packages.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from visualization_msgs.msg import MarkerArray
import cv2
from cv_bridge import CvBridge
import numpy as np
import message_filters


class IsaacROSVisualSLAMNode(Node):
    """
    Isaac ROS Visual SLAM Node
    Demonstrates GPU-accelerated Visual SLAM using Isaac ROS packages
    """

    def __init__(self):
        super().__init__('isaac_ros_vslam_node')

        # Parameters
        self.declare_parameter('use_gpu', True)
        self.declare_parameter('image_width', 1920)
        self.declare_parameter('image_height', 1080)
        self.declare_parameter('max_features', 1000)
        self.declare_parameter('feature_threshold', 20)

        self.use_gpu = self.get_parameter('use_gpu').value
        self.image_width = self.get_parameter('image_width').value
        self.image_height = self.get_parameter('image_height').value
        self.max_features = self.get_parameter('max_features').value
        self.feature_threshold = self.get_parameter('feature_threshold').value

        # CV Bridge for image conversion
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )

        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )

        self.imu_sub = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )

        # Publishers
        self.odom_pub = self.create_publisher(Odometry, '/visual_slam/odometry', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/visual_slam/pose', 10)
        self.feature_pub = self.create_publisher(MarkerArray, '/visual_slam/features', 10)

        # Internal state
        self.latest_image = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.prev_features = None
        self.prev_gray = None
        self.position = np.array([0.0, 0.0, 0.0])
        self.orientation = np.array([0.0, 0.0, 0.0, 1.0])  # quaternion

        # Feature detector (using GPU if available)
        if self.use_gpu and cv2.cuda.getCudaEnabledDeviceCount() > 0:
            self.feature_detector = cv2.cuda.SURF_create(self.feature_threshold)
            self.get_logger().info("Using GPU-accelerated feature detection")
        else:
            self.feature_detector = cv2.xfeatures2d.SURF_create(self.feature_threshold)
            self.get_logger().info("Using CPU-based feature detection")

        self.get_logger().info("Isaac ROS Visual SLAM node initialized")

    def image_callback(self, msg):
        """Process incoming image messages"""
        try:
            # Convert ROS Image to OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Store the latest image for processing
            self.latest_image = cv_image.copy()

            # Process VSLAM if we have camera parameters
            if self.camera_matrix is not None:
                self.process_vslam(cv_image)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

    def camera_info_callback(self, msg):
        """Process camera info messages"""
        self.camera_matrix = np.array(msg.k).reshape(3, 3)
        self.dist_coeffs = np.array(msg.d)

    def imu_callback(self, msg):
        """Process IMU messages for sensor fusion"""
        # Extract orientation from IMU
        self.orientation[0] = msg.orientation.x
        self.orientation[1] = msg.orientation.y
        self.orientation[2] = msg.orientation.z
        self.orientation[3] = msg.orientation.w

    def process_vslam(self, image):
        """Main VSLAM processing function"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Detect features
        if self.prev_features is None:
            # First frame - detect initial features
            self.prev_features = self.detect_features(gray)
            self.prev_gray = gray.copy()
        else:
            # Track features from previous frame
            curr_features = self.detect_features(gray)

            if len(self.prev_features) > 0 and len(curr_features) > 0:
                # Compute optical flow to track features
                prev_pts = np.float32([f.pt for f in self.prev_features]).reshape(-1, 1, 2)
                curr_pts = np.float32([f.pt for f in curr_features]).reshape(-1, 1, 2)

                # Calculate motion
                if len(prev_pts) >= 8 and len(curr_pts) >= 8:
                    # Estimate motion using essential matrix
                    E, mask = cv2.findEssentialMat(
                        curr_pts, prev_pts,
                        self.camera_matrix,
                        method=cv2.RANSAC,
                        threshold=1.0
                    )

                    if E is not None:
                        # Recover pose
                        _, R, t, mask_pose = cv2.recoverPose(E, curr_pts, prev_pts, self.camera_matrix)

                        # Update position (simplified)
                        self.position += t.flatten() * 0.1  # Scale factor for visualization

                        # Publish odometry
                        self.publish_odometry()

            # Update for next iteration
            self.prev_features = curr_features
            self.prev_gray = gray.copy()

    def detect_features(self, gray_image):
        """Detect features in the image"""
        if self.use_gpu and cv2.cuda.getCudaEnabledDeviceCount() > 0:
            # GPU processing
            gpu_img = cv2.cuda_GpuMat()
            gpu_img.upload(gray_image)

            keypoints = self.feature_detector.detect(gpu_img)
            # Convert keypoints back to CPU for processing
            keypoints_cpu = []
            for kp in keypoints:
                keypoints_cpu.append(cv2.KeyPoint(kp.pt[0], kp.pt[1], kp.size))
            return keypoints_cpu
        else:
            # CPU processing
            return self.feature_detector.detect(gray_image)

    def publish_odometry(self):
        """Publish odometry information"""
        odom_msg = Odometry()
        odom_msg.header.stamp = self.get_clock().now().to_msg()
        odom_msg.header.frame_id = 'map'
        odom_msg.child_frame_id = 'camera'

        # Position
        odom_msg.pose.pose.position.x = float(self.position[0])
        odom_msg.pose.pose.position.y = float(self.position[1])
        odom_msg.pose.pose.position.z = float(self.position[2])

        # Orientation (from IMU)
        odom_msg.pose.pose.orientation.x = float(self.orientation[0])
        odom_msg.pose.pose.orientation.y = float(self.orientation[1])
        odom_msg.pose.pose.orientation.z = float(self.orientation[2])
        odom_msg.pose.pose.orientation.w = float(self.orientation[3])

        # Publish
        self.odom_pub.publish(odom_msg)

        # Also publish as PoseStamped
        pose_msg = PoseStamped()
        pose_msg.header = odom_msg.header
        pose_msg.pose = odom_msg.pose.pose
        self.pose_pub.publish(pose_msg)


def main(args=None):
    rclpy.init(args=args)

    vslam_node = IsaacROSVisualSLAMNode()

    try:
        rclpy.spin(vslam_node)
    except KeyboardInterrupt:
        pass
    finally:
        vslam_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()