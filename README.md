# ORB-SLAM3 - Minimal Library Build


> **Note:** This is a simplified version focused on building only the shared library. For complete documentation including examples, ROS integration, and dataset instructions, see the [original ORB-SLAM3 README](https://github.com/UZ-SLAMLab/ORB_SLAM3).

### Authors
Carlos Campos, Richard Elvira, Juan J. Gómez Rodríguez, José M. M. Montiel, Juan D. Tardos

### Reference
**ORB-SLAM3: An Accurate Open-Source Library for Visual, Visual-Inertial and Multi-Map SLAM**, *IEEE Transactions on Robotics 37(6):1874-1890, Dec. 2021*. [PDF](https://arxiv.org/abs/2007.11898)

## License

ORB-SLAM3 is released under [GPLv3 license](https://github.com/UZ-SLAMLab/ORB_SLAM3/LICENSE). See [Dependencies.md](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Dependencies.md) for dependencies and licenses.

If you use ORB-SLAM3 in academic work, please cite:
```bibtex
@article{ORBSLAM3_TRO,
  title={{ORB-SLAM3}: An Accurate Open-Source Library for Visual, Visual-Inertial 
         and Multi-Map {SLAM}},
  author={Campos, Carlos AND Elvira, Richard AND G\'omez, Juan J. AND Montiel, 
          Jos\'e M. M. AND Tard\'os, Juan D.},
  journal={IEEE Transactions on Robotics}, 
  volume={37},
  number={6},
  pages={1874-1890},
  year={2021}
}
```

## Prerequisites

### Required Dependencies

#### C++14 Compiler
A C++14 compatible compiler is required.

#### Pangolin
For visualization and user interface.
```bash
# Install dependencies
sudo apt install libgl1-mesa-dev libglew-dev cmake
git clone https://github.com/stevenlovegrove/Pangolin.git
cd Pangolin
mkdir build && cd build
cmake ..
make -j4
sudo make install
```

#### Core Dependencies
Install all required libraries:
```bash
# OpenCV (>= 4.4), Eigen3 (>= 3.1.0), Boost, OpenSSL
sudo apt install libopencv-dev libeigen3-dev libboost-all-dev libssl-dev

# Python with OpenCV for camera calibration
sudo apt install python3-opencv python3-numpy python3-yaml
```

## Building the Library

Clone the repository:
```bash
git clone https://github.com/UZ-SLAMLab/ORB_SLAM3.git ORB_SLAM3
cd ORB_SLAM3
```

Build everything (third-party libraries + ORB-SLAM3):
```bash


cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..


chmod +x build.sh
./build.sh
```

This creates:
- **libORB_SLAM3.so** in the `lib/` folder
- Uncompressed vocabulary file in `Vocabulary/`

## Quick Start: ROS2 Integration

A ROS2 monocular wrapper is available for easy integration with ROS2 robotics systems:

```bash
# Setup ROS2 workspace
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
ln -s ~/Projects/ORB_SLAM3/ROS2 orb_slam3_ros2

# Build
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --packages-select orb_slam3_ros2
source install/setup.bash

# Run with webcam
ros2 launch orb_slam3_ros2 mono_webcam.launch.py
```

The ROS2 wrapper provides:
- **Monocular camera** support
- **Camera pose publisher** (`geometry_msgs/PoseStamped`)
- **Trajectory path publisher** (`nav_msgs/Path`)
- **TF transforms** (world → camera)
- **RViz2 visualization**
- **Webcam launch file** for quick testing

See `ROS2/README.md` for complete documentation, topics, and configuration.



## Additional Resources

- **Original Documentation**: [ORB-SLAM3 GitHub](https://github.com/UZ-SLAMLab/ORB_SLAM3)
- **Changelog**: [CHANGELOG.md](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/master/Changelog.md)
- **Paper**: [arXiv:2007.11898](https://arxiv.org/abs/2007.11898)
- **Calibration Tutorial**: See `Calibration_Tutorial.pdf` in the repository
