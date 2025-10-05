# ORB-SLAM3 ROS2 Wrapper

ROS2 wrapper for ORB-SLAM3 with monocular camera support.

## Features

- **Monocular SLAM**: Single camera support
- **ROS2 Publishers**:
  - Camera pose (`geometry_msgs/PoseStamped`)
  - Camera trajectory path (`nav_msgs/Path`)
  - TF transforms (camera pose in world frame)
- **Automatic trajectory saving** on shutdown
- **Webcam launch file** for easy testing

## Prerequisites

### ROS2 Installation
Tested with **ROS2 Humble** on Ubuntu 22.04.

Install ROS2 if not already installed:
```bash
# Follow: https://docs.ros.org/en/humble/Installation.html
```

### Required ROS2 Packages
```bash
sudo apt install ros-humble-cv-bridge ros-humble-image-transport \
                 ros-humble-message-filters ros-humble-tf2-ros \
                 ros-humble-usb-cam
```

### ORB-SLAM3 Library
Make sure ORB-SLAM3 is built:
```bash
cd ~/Projects/ORB_SLAM3
./build.sh
```

## Building

### Setup ROS2 Workspace
```bash
# Create a ROS2 workspace if you don't have one
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src

# Symlink this package
ln -s ~/Projects/ORB_SLAM3/ROS2 orb_slam3_ros2

cd ~/ros2_ws
```

### Build the Package
```bash
# Source ROS2
source /opt/ros/humble/setup.bash

# Build
colcon build --packages-select orb_slam3_ros2

# Source the workspace
source install/setup.bash
```

## Configuration

### ROS2 Parameters

The node can be configured via parameters in `config/params.yaml`:

```yaml
orb_slam3_mono:
  ros__parameters:
    # Topic names
    image_topic: "camera/image_raw"
    camera_info_topic: "camera/camera_info"
    pose_topic: "orb_slam3/camera_pose"
    path_topic: "orb_slam3/camera_path"
    
    # Frame IDs
    world_frame_id: "world"
    camera_frame_id: "camera"
    
    # Publisher settings
    queue_size: 10
    publish_tf: true
```

### Camera Calibration

Camera intrinsics can be provided in two ways:

1. **Via camera_info topic** (recommended for ROS2):
   - The node subscribes to `/camera/camera_info`
   - Most ROS2 camera drivers publish this automatically
   - Intrinsics are logged when first received

2. **Via ORB-SLAM3 config file** (`config/webcam.yaml`):
   - Traditional ORB-SLAM3 YAML format
   - Required for ORB feature extraction parameters

## Usage

### Monocular with Webcam

**Launch with default settings:**
```bash
source ~/ros2_ws/install/setup.bash
ros2 launch orb_slam3_ros2 mono_webcam.launch.py
```

**Launch with custom parameters:**
```bash
ros2 launch orb_slam3_ros2 mono_webcam.launch.py \
  params:=/path/to/custom_params.yaml \
  vocabulary:=/path/to/vocabulary.txt \
  settings:=/path/to/camera_settings.yaml \
  use_viewer:=true
```

**Run node separately:**
```bash
ros2 run orb_slam3_ros2 mono \
  ~/Projects/ORB_SLAM3/Vocabulary/ORBvoc.txt \
  ~/ros2_ws/src/orb_slam3_ros2/config/webcam.yaml \
  true \
  --ros-args --params-file ~/ros2_ws/src/orb_slam3_ros2/config/params.yaml
```

## Topics

### Subscribed Topics

- `/camera/image_raw` (sensor_msgs/Image) - Monocular camera input
- `/camera/camera_info` (sensor_msgs/CameraInfo) - Camera intrinsics (optional, informational)

### Published Topics

- `/orb_slam3/camera_pose` (geometry_msgs/PoseStamped) - Current camera pose in world frame
- `/orb_slam3/camera_path` (nav_msgs/Path) - Full trajectory path
- `/tf` - Transform from `world` to `camera` frame (if `publish_tf` is enabled)

**Note:** Topic names can be customized via ROS2 parameters in `config/params.yaml`.

## Visualization

### RViz2
```bash
ros2 run rviz2 rviz2
```

Add these displays:
1. **Path**: Topic `/orb_slam3/camera_path`, Fixed Frame `world`
2. **PoseStamped**: Topic `/orb_slam3/camera_pose`
3. **TF**: Show camera frame

### Pangolin Viewer
Set the third argument to `true` when running nodes (default):
```bash
ros2 run orb_slam3_ros2 mono <vocab> <settings> true
```

## Configuration

Camera calibration files should be in YAML format. Example in `config/webcam.yaml`.

### Important Parameters

```yaml
Camera.type: "PinHole"
Camera.fx: 500.0      # Focal length x
Camera.fy: 500.0      # Focal length y
Camera.cx: 320.0      # Principal point x
Camera.cy: 240.0      # Principal point y
Camera.k1: 0.0        # Radial distortion k1
Camera.k2: 0.0        # Radial distortion k2
Camera.p1: 0.0        # Tangential distortion p1
Camera.p2: 0.0        # Tangential distortion p2

Camera.width: 640
Camera.height: 480
Camera.fps: 30.0
```

**Important:** Calibrate your camera for accurate results!

## Camera Calibration

Use ROS2 camera calibration:
```bash
ros2 run camera_calibration cameracalibrator \
  --size 8x6 --square 0.025 \
  image:=/camera/image_raw camera:=/camera
```

Or use the OpenCV calibration tools.

## Output Files

When the node shuts down (Ctrl+C), it saves:
- `CameraTrajectory.txt` - Full camera trajectory (TUM format)
- `KeyFrameTrajectory.txt` - Keyframe trajectory (TUM format)

## Example: Testing with Webcam

1. **Build everything:**
   ```bash
   cd ~/Projects/ORB_SLAM3
   ./build.sh
   
   cd ~/ros2_ws
   colcon build --packages-select orb_slam3_ros2
   source install/setup.bash
   ```

2. **Launch:**
   ```bash
   ros2 launch orb_slam3_ros2 mono_webcam.launch.py
   ```

3. **Visualize in RViz2 (optional):**
   ```bash
   # In another terminal
   ros2 run rviz2 rviz2
   # Add Path display with topic /orb_slam3/camera_path
   ```

4. **Move your camera** slowly to initialize tracking

5. **Stop with Ctrl+C** - trajectories will be saved

## Troubleshooting

### No images received
```bash
# Check if camera is publishing
ros2 topic list
ros2 topic hz /camera/image_raw
ros2 topic echo /camera/image_raw --no-arr
```

### Camera not found
```bash
# List available cameras
ls /dev/video*

# Try different camera ID
ros2 run usb_cam usb_cam_node_exe --ros-args -p video_device:=/dev/video1
```

### Build errors
```bash
# Make sure ORB-SLAM3 library exists
ls ~/Projects/ORB_SLAM3/lib/libORB_SLAM3.so

# Source ROS2 properly
source /opt/ros/humble/setup.bash
```

### Poor tracking
- Calibrate your camera properly
- Ensure good lighting
- Move camera slowly during initialization
- Check that feature count is appropriate in config file

## Advanced Usage

### Custom Topics via Parameters

Edit `config/params.yaml` to customize topic names:

```yaml
orb_slam3_mono:
  ros__parameters:
    image_topic: "/my_robot/camera/image"
    camera_info_topic: "/my_robot/camera/info"
    pose_topic: "/slam/pose"
    path_topic: "/slam/path"
    world_frame_id: "map"
    camera_frame_id: "camera_optical_frame"
```

Or override at launch time:
```bash
ros2 run orb_slam3_ros2 mono \
  <vocab> <settings> true \
  --ros-args \
  -p image_topic:=/my_camera/image \
  -p camera_info_topic:=/my_camera/info \
  -p world_frame_id:=map \
  -p publish_tf:=false
```

### Disable TF Publishing

To disable TF broadcasting (useful if you have your own localization):
```yaml
publish_tf: false
```

### Disable Viewer

Set the viewer argument to `false` for headless operation:
```bash
ros2 run orb_slam3_ros2 mono <vocab> <settings> false
```

### Record a Bag File

```bash
# Record camera images
ros2 bag record /camera/image_raw

# Play back
ros2 bag play <bagfile>
```

## Integration with Your Robot

To integrate with your robot:

1. **Match topic names** in your launch file or use remapping
2. **Calibrate your camera** and create a config YAML
3. **Test with recorded data** first (bag files)
4. **Tune ORB parameters** for your environment
5. **Use published pose** for navigation or other tasks

## Code Structure

```
ROS2/
├── src/
│   └── monocular_node.cpp    # Monocular SLAM node
├── launch/
│   └── mono_webcam.launch.py # Launch file for webcam
├── config/
│   └── webcam.yaml           # Camera configuration
├── CMakeLists.txt            # Build configuration
├── package.xml               # ROS2 package manifest
└── README.md                 # This file
```

## References

- [ORB-SLAM3 Paper](https://arxiv.org/abs/2007.11898)
- [ROS2 Documentation](https://docs.ros.org/en/humble/)
- [cv_bridge Tutorial](https://docs.ros.org/en/humble/p/cv_bridge/)
