# robot-follower-yolov8
This project uses ROS 2 Humble and YOLOv8 for a robot follower system. It detects and tracks a person in real time, calculates distances using depth camera data, and controls the robot's motion to follow or stop based on proximity. Features include precise velocity control, smooth movement, and RViz visualization for target tracking.

# Robot Follower with YOLOv8

This project implements a robot follower using ROS 2 (Humble) and YOLOv8 for person detection.

## Features
- Detects and follows a person within a specified range using YOLOv8.
- Uses depth data to determine distance and angular adjustments.
- Publishes velocity commands and RViz markers for visualization.

## Setup Instructions
1. Clone the repository:
2. Install dependencies:
3. Run the node:


## Dependencies
- Python 3.10+
- ROS 2 Humble
- YOLOv8
- CvBridge

