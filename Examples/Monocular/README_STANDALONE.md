# Standalone Build for mono_tum Example

This directory contains a standalone CMakeLists.txt for building only the `mono_tum` example without rebuilding the entire ORB-SLAM3 project.

## Prerequisites

Before building this example, you must have:
1. Built the main ORB-SLAM3 library (`libORB_SLAM3.so` in `../../lib/`)
2. Installed all dependencies (OpenCV 4.4+, Eigen3, Pangolin)

## Building

Simply run the build script:

```bash
./build.sh
```

Or manually:

```bash
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
```

## Output

The executable `mono_tum` will be created in this directory (Examples/Monocular/).

## Usage

```bash
./mono_tum path_to_vocabulary path_to_settings path_to_sequence
```

### Example:

```bash
./mono_tum ../../Vocabulary/ORBvoc.txt TUM1.yaml /path/to/tum/dataset/rgbd_dataset_freiburg1_xyz
```

## Notes

- This CMakeLists.txt uses C++14 standard (required for Pangolin)
- The executable links against the pre-built `libORB_SLAM3.so`
- If you modify the ORB-SLAM3 library source code, rebuild the main library first before rebuilding this example
- Compiler warnings are suppressed to match the main project configuration

## Cleaning

To clean the build:

```bash
rm -rf build mono_tum
```
