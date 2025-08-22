# Robot Follower with YOLOv8

This project implements a robot follower using ROS 2 (Humble) and YOLOv8 for person detection. It detects and tracks a person in real time, calculates distances using depth camera data, and controls the robot's motion to follow or stop based on proximity. Features include precise velocity control, smooth movement, and RViz visualization for target tracking.

## Features
- **Real-time person detection** using YOLOv8 with high accuracy and speed (~8-12ms inference time)
- **Distance-based following** with configurable follow range (0.5m - 2.0m)
- **Smooth motion control** with exponential smoothing for stable movement
- **RViz visualization** with markers for target tracking
- **Multiple camera support** - works with RealSense D435i and other RGB-D cameras
- **ROS 2 integration** with standard topics for easy integration

## Hardware Requirements
- RGB-D Camera (tested with Intel RealSense D435i)
- Robot platform capable of differential drive
- Computer with CUDA-compatible GPU (recommended for optimal performance)

## Software Dependencies
- **Python 3.10+**
- **ROS 2 Humble**
- **YOLOv8 (ultralytics)**
- **OpenCV**
- **NumPy 1.24.2** (for ROS 2 compatibility)
- **cv_bridge**
- **RealSense SDK** (for Intel RealSense cameras)

## Dependency Files
This project includes several dependency files for different use cases:

- `requirements.txt` - Complete frozen dependencies with exact versions for full environment reproduction
- `requirements-minimal.txt` - Minimal dependencies (only main packages, dependencies auto-resolved)
- `pyproject.toml` - Modern Python packaging with uv native dependency management

## Installation

### 1. Clone the repository
```bash
git clone https://github.com/ahmadidrisyakubu/robot-follower-yolov8.git
cd robot-follower-yolov8
```

### 2. Set up virtual environment with uv
```bash
# Install uv if not already installed
curl -LsSf https://astral.sh/uv/install.sh | sh

# Create Python 3.10 virtual environment
uv venv --python 3.10 robot-follower-env-310

# Option A: Install minimal dependencies (recommended)
uv pip install --python robot-follower-env-310/bin/python -r requirements-minimal.txt

# Option B: Install exact dependencies (complete environment reproduction)
uv pip install --python robot-follower-env-310/bin/python -r requirements.txt

# Option C: Install using pyproject.toml (uv native)
uv sync
```

### 3. Set up RealSense camera (if using Intel RealSense D435i)
```bash
# Install RealSense SDK
sudo apt update
sudo apt install ros-humble-realsense2-*

# Launch RealSense camera
ros2 launch realsense2_camera rs_launch.py
```

## Usage

### 1. Start the RealSense camera
```bash
# In terminal 1
ros2 launch realsense2_camera rs_launch.py
```

### 2. Run the robot follower
```bash
# In terminal 2
source /opt/ros/humble/setup.bash
robot-follower-env-310/bin/python Robot_follower.py
```

### 3. Monitor the system
You can monitor the robot's behavior using various ROS 2 tools:

#### Check person detection
```bash
# Filter logs to see person detections
ros2 run robot-follower-yolov8 Robot_follower.py 2>&1 | grep "person"
```

#### Monitor velocity commands
```bash
# Check robot velocity commands
ros2 topic echo /cmd_vel
```

#### Visualize in RViz
```bash
# Launch RViz for visualization
rviz2
# Add marker topic: /visualization_marker
```

#### Check camera topics
```bash
# List available camera topics
ros2 topic list | grep camera

# Monitor camera data
ros2 topic hz /camera/camera/color/image_raw
ros2 topic hz /camera/camera/depth/image_rect_raw
```

## Configuration

The robot follower can be configured by modifying parameters in `Robot_follower.py`:

### Following Parameters
```python
self.min_follow_distance = 0.5  # Minimum distance to follow (meters)
self.max_follow_distance = 2.0  # Maximum distance to stop following (meters)
```

### Velocity Control
```python
self.linear_velocity_scale = 0.1   # Linear velocity scaling factor
self.angular_velocity_scale = 2.0  # Angular velocity scaling factor
```

### Detection Parameters
```python
self.confidence_threshold = 0.1      # Minimum confidence for YOLO detections
self.distance_threshold = 50         # Maximum distance (pixels) to associate bounding boxes
```

### Smoothing Parameters
```python
self.omega = 0.3  # Filtering coefficient for exponential smoothing (0-1)
```

## ROS 2 Topics

### Subscribed Topics
- `/camera/camera/color/image_raw` (sensor_msgs/Image) - RGB camera feed
- `/camera/camera/depth/image_rect_raw` (sensor_msgs/Image) - Depth camera feed
- `/camera/camera/color/camera_info` (sensor_msgs/CameraInfo) - Camera calibration data

### Published Topics
- `/cmd_vel` (geometry_msgs/Twist) - Robot velocity commands
- `/visualization_marker` (visualization_msgs/Marker) - RViz visualization markers

## Troubleshooting

### Person detected but robot not moving
1. Check if person is within follow range (0.5m - 2.0m)
2. Verify depth data is valid: `ros2 topic echo /camera/camera/depth/image_rect_raw`
3. Check velocity commands: `ros2 topic echo /cmd_vel`

### Camera topics not found
1. Ensure RealSense camera is connected and launched
2. Check available topics: `ros2 topic list | grep camera`
3. Verify camera permissions and USB connection

### YOLO model download issues
The YOLOv8 model (`yolov8m-seg.pt`) will be automatically downloaded on first run. If download fails:
1. Check internet connection
2. Manually download from: https://github.com/ultralytics/assets/releases/download/v8.3.0/yolov8m-seg.pt

### NumPy compatibility issues
If you encounter NumPy version conflicts:
```bash
uv pip install --python robot-follower-env-310/bin/python "numpy==1.24.2" --force-reinstall
```

## Performance Optimization

### For better performance:
1. **Use GPU acceleration**: Ensure CUDA is properly installed for YOLOv8 GPU inference
2. **Adjust image resolution**: Lower camera resolution for faster processing
3. **Tune confidence threshold**: Higher threshold reduces false positives but may miss detections
4. **Optimize filtering**: Adjust `omega` parameter for smoother but potentially slower response

## Safety Considerations

1. **Test in safe environment**: Always test in a clear, safe area
2. **Emergency stop**: Keep manual control available to stop the robot
3. **Obstacle avoidance**: This system only follows people - add obstacle avoidance for real deployments
4. **Speed limits**: Keep velocity scales low for safe operation

## License

This project is licensed under the BSD 3-Clause License - see the LICENSE file for details.

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## Support

For issues and questions:
1. Check the troubleshooting section above
2. Review ROS 2 and YOLOv8 documentation
3. Create an issue on the GitHub repository

