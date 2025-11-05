# FIDUCIAL ANGLE TRACKING
A ROS2 package for detecting AprilTags and calculating relative poses between excavator components using computer vision.

## Overview

This package detects AprilTags (tag36h11 family) from camera images and calculates the relative transformations between different parts of an excavator system. It publishes TF transformations and optionally displays annotated images with detection results and relative angles.

## Features

- **AprilTag Detection**: Detects tags using the dt_apriltags library
- **Pose Estimation**: Calculates 6DOF pose for each detected tag
- **Relative Transformations**: Computes relative poses between kinematic chain components
- **TF Broadcasting**: Publishes transforms to the TF tree for each detected tag
- **Joint State Publishing**: Converts TF transforms to JointState messages with yaw angles
- **Visual Feedback**: Optional annotated image output with angles and coordinate frames
- **Configurable Parameters**: Configurable tag IDs, names, and display options
- **Kinematic Chain Processing**: Automatically processes kinematic chain relationships

## System Components

The package tracks the following excavator components with their corresponding tag IDs:

| Component | Tag ID | Description |
|-----------|---------|-------------|
| Cabin     | 9       | Base/cabin of excavator |
| Boom      | 12      | First joint (boom) |
| Arm       | 20      | Second joint (arm) |
| Bucket    | 10      | End effector (bucket) |

## Kinematic Chain

The system establishes the following kinematic relationships:
- Cabin → Boom
- Boom → Arm  
- Arm → Bucket

## Installation

1. Build the packages:
```bash
colcon build --packages-select aruco_detector_pkg joint_state_publisher
source install/setup.bash
```

## Usage

### Basic Launch Commands

#### Launch Both Packages Together
```bash
# Terminal 1: Start the detector
ros2 launch aruco_detector_pkg aruco_detector.launch.py

# Terminal 2: Start the joint state publisher
ros2 launch joint_state_publisher joint_state_publisher.launch.py
```


### Published Topics

- `/tf` - Transform tree with tag poses relative to camera_link
- `/gt_joint_states` - Joint angles in degrees (yaw rotation between consecutive tags)
- `/apriltag_detector/annotated_image` - Annotated camera image (when display_detections=true)

### Parameter Configuration

#### AruCo Detector Parameters

Edit `src/aruco_detector_pkg/config/params.yaml`:
```yaml
aruco_detector_node:
  ros__parameters:
    display_detections: true  # Set to false to disable visual annotations
```


## Visualization

### 1. Real-time Joint Angles

Monitor the joint angles being published:
```bash
ros2 topic echo /gt_joint_states
```

### 3. RViz Visualization

For 3D visualization of the transforms and annotated image:

```bash
# Terminal 1: Launch the detector (if not already running)
ros2 launch aruco_detector_pkg aruco_detector.launch.py

# Terminal 2: Launch RViz
rviz2
```

In RViz:
1. Set **Fixed Frame** to `camera_link`
2. Add **TF** display to visualize coordinate frames
3. Add **Camera** display and set topic to `/apriltag_detector/annotated_image`




