#!/usr/bin/env python3
"""
Robot Follower with YOLOv8 and ROS 2
This script uses YOLOv8 for object detection to identify and track a person in the robot's field of view.
It calculates the 3D position of the detected person using depth data and commands the robot to follow them
within a specified distance range.
"""

# Import necessary libraries
import rclpy  # ROS 2 Python client library
from rclpy.node import Node  # Base class for ROS 2 nodes
from sensor_msgs.msg import Image, CameraInfo  # Message types for image and camera information
from geometry_msgs.msg import Twist  # Message type for robot velocity commands
from cv_bridge import CvBridge  # Bridge to convert ROS images to OpenCV format
import numpy as np  # For numerical operations
from ultralytics import YOLO  # YOLOv8 object detection library
from visualization_msgs.msg import Marker  # For RViz visualization
import time  # Used for tracking time differences

# Define the main ROS 2 Node
class PersonDistanceCalculator(Node):
    def __init__(self):
        # Initialize the Node with a unique name
        super().__init__('person_distance_calculator')

        # Initialize YOLO model using a pre-trained checkpoint
        # Replace 'yolov8m-seg.pt' with your own model file path if needed
        self.model = YOLO('yolov8m-seg.pt')  
        self.get_logger().info("YOLO model loaded")

        # Initialize CvBridge for converting ROS image messages to OpenCV images
        self.bridge = CvBridge()
        self.get_logger().info("CvBridge initialized")

        # Variables to hold the depth image and camera information
        self.depth_image = None
        self.camera_info = None

        # Used for tracking the currently targeted bounding box
        self.target_bbox = None

        # Variables for controlling the robot's velocity and smooth motion
        self.x_filtered = 0.0  # Smoothed x-coordinate
        self.y_filtered = 0.0  # Smoothed y-coordinate
        self.omega = 0.3  # Filtering coefficient for exponential smoothing

        # Define velocity scaling factors for linear and angular motion
        self.linear_velocity_scale = 0.1  # Slow linear velocity
        self.angular_velocity_scale = 2.0  # High angular velocity for fast turning

        # Threshold values for filtering and person detection
        self.confidence_threshold = 0.1  # Minimum confidence for YOLO detections
        self.distance_threshold = 50  # Maximum distance (pixels) to associate bounding boxes
        self.min_follow_distance = 0.5  # Minimum distance to follow a person (meters)
        self.max_follow_distance = 2.0  # Maximum distance to stop following (meters)

        # Publishers and Subscribers for ROS topics
        self.marker_pub = self.create_publisher(Marker, "/visualization_marker", 2)  # For RViz visualization
        self.cmd_vel_pub = self.create_publisher(Twist, "/cmd_vel", 2)  # For robot velocity commands

        # Subscribe to depth image, RGB image, and camera info topics
        self.depth_sub = self.create_subscription(Image, '/camera/depth/image_raw', self.depth_callback, 2)
        self.rgb_sub = self.create_subscription(Image, '/camera/color/image_raw', self.rgb_callback, 2)
        self.info_sub = self.create_subscription(CameraInfo, '/camera/color/camera_info', self.camera_info_callback, 2)

        # Log that all subscriptions and publishers are set up
        self.get_logger().info("Subscriptions and Publishers initialized")

    def depth_callback(self, depth_msg):
        """
        Callback function for depth images.
        Converts the depth image to a format that can be processed.
        """
        self.depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')  # Convert to OpenCV format

    def camera_info_callback(self, info_msg):
        """
        Callback function to receive and store camera intrinsic parameters.
        These are required for projecting 2D image points into 3D space.
        """
        self.camera_info = info_msg

    def rgb_callback(self, rgb_msg):
        """
        Callback function for RGB images.
        Performs YOLO detection on the image, calculates the 3D position of detected people,
        and determines if the robot should follow the person or stop.
        """
        # Ensure depth image and camera info are available before processing
        if self.depth_image is None or self.camera_info is None:
            self.get_logger().info("Waiting for depth image and camera info...")
            return

        # Convert the ROS Image message to an OpenCV image
        rgb_image = self.bridge.imgmsg_to_cv2(rgb_msg, desired_encoding='bgr8')

        # Perform object detection using YOLOv8
        results = self.model(rgb_image, conf=self.confidence_threshold)
        self.get_logger().info("YOLO detection results obtained")

        # Variables to track the closest detected person
        closest_bbox = None
        min_dist = float('inf')

        # Process detection results
        if results:
            for result in results:
                for box in result.boxes:
                    if box.cls == 0:  # Check if detected object is a person (class ID 0 for YOLO)
                        # Extract bounding box coordinates
                        x1, y1, x2, y2 = map(int, box.xyxy[0])  
                        bbox_center = ((x1 + x2) // 2, (y1 + y2) // 2)  # Center of bounding box

                        # Calculate distance from the current bounding box to the previously targeted one
                        if self.target_bbox is not None:
                            target_center = ((self.target_bbox[0] + self.target_bbox[2]) // 2,
                                             (self.target_bbox[1] + self.target_bbox[3]) // 2)
                            dist = np.linalg.norm(np.array(bbox_center) - np.array(target_center))

                            # Update the closest bounding box if it is within the threshold
                            if dist < min_dist and dist < self.distance_threshold:
                                min_dist = dist
                                closest_bbox = (x1, y1, x2, y2)

            # Update target bounding box if a valid detection was found
            if closest_bbox is not None:
                self.target_bbox = closest_bbox
                self.calculate_and_publish_velocity()

    def calculate_and_publish_velocity(self):
        """
        Calculates the robot's velocity based on the position of the detected person
        and publishes the velocity commands to the robot.
        """
        # Extract the center of the target bounding box
        cx = (self.target_bbox[0] + self.target_bbox[2]) // 2
        cy = (self.target_bbox[1] + self.target_bbox[3]) // 2

        try:
            # Retrieve the depth value at the bounding box center
            depth_value = self.depth_image[cy, cx]
        except IndexError:
            self.get_logger().error(f"Depth image index ({cy}, {cx}) is out of bounds")
            return

        # Ignore invalid depth values
        if np.isnan(depth_value) or depth_value <= 0:
            self.get_logger().warn("Invalid depth value at target center")
            return

        # Convert depth from millimeters to meters
        distance = depth_value / 1000.0

        # Use camera intrinsics to compute 3D coordinates
        fx, fy = self.camera_info.k[0], self.camera_info.k[4]  # Focal lengths
        cx_cam, cy_cam = self.camera_info.k[2], self.camera_info.k[5]  # Principal points
        x = distance
        y = -(cx - cx_cam) * distance / fx

        # Apply exponential smoothing to reduce sudden motion changes
        self.x_filtered = self.omega * self.x_filtered + (1 - self.omega) * x
        self.y_filtered = self.omega * self.y_filtered + (1 - self.omega) * y

        # Publish velocity commands if the person is within the follow range
        if self.min_follow_distance <= distance <= self.max_follow_distance:
            linear_velocity = self.x_filtered * self.linear_velocity_scale
            angular_velocity = self.y_filtered * self.angular_velocity_scale
            twist = Twist()
            twist.linear.x = linear_velocity
            twist.angular.z = angular_velocity
            self.cmd_vel_pub.publish(twist)
        else:
            # Stop the robot if the person is outside the follow range
            self.cmd_vel_pub.publish(Twist())

def main():
    rclpy.init()  # Initialize the ROS 2 Python client library
    node = PersonDistanceCalculator()  # Create an instance of the Node
    rclpy.spin(node)  # Spin the Node to keep it running
    node.destroy_node()  # Clean up the Node before exiting
    rclpy.shutdown()  # Shutdown the ROS 2 client library

# Run the main function if this script is executed
if __name__ == '__main__':
    main()
