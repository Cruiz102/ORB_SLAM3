# Running mono_tum_vi with TUM-VI Dataset

## Quick Start

### 1. Download TUM-VI Dataset

The TUM-VI (Technical University of Munich - Visual-Inertial) dataset is designed for visual-inertial odometry and SLAM.

**Download from:** https://vision.in.tum.de/data/datasets/visual-inertial-dataset

Popular sequences:
- **room1** - Small indoor room (easiest)
- **room2** - Larger room with more features
- **room3** - Large room with difficult lighting
- **room4** - Very large room
- **room5** - Long corridor
- **room6** - Large hall

Example download:
```bash
# Create dataset directory
mkdir -p ~/Datasets/TUM_VI

# Download a sequence (example: room1)
cd ~/Datasets/TUM_VI
wget https://vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-room1_512_16.tar
tar -xf dataset-room1_512_16.tar
```

### 2. Dataset Structure

After extraction, you should have:
```
dataset-room1_512_16/
├── mav0/
│   ├── cam0/
│   │   ├── data/           # Camera images
│   │   └── data.csv        # Timestamps
│   ├── imu0/
│   └── ...
```

### 3. Run mono_tum_vi

```bash
cd /home/cesar/Projects/ORB_SLAM3/Examples/Monocular

# Basic usage (single sequence):
./mono_tum_vi \
    ../../Vocabulary/ORBvoc.txt \
    TUM-VI.yaml \
    ~/Datasets/TUM_VI/dataset-room1_512_16/mav0/cam0/data \
    ~/Datasets/TUM_VI/dataset-room1_512_16/mav0/cam0/data.csv
```

### 4. Multiple Sequences (Advanced)

You can process multiple sequences in one run:

```bash
./mono_tum_vi \
    ../../Vocabulary/ORBvoc.txt \
    TUM-VI.yaml \
    ~/Datasets/TUM_VI/dataset-room1_512_16/mav0/cam0/data \
    ~/Datasets/TUM_VI/dataset-room1_512_16/mav0/cam0/data.csv \
    ~/Datasets/TUM_VI/dataset-room2_512_16/mav0/cam0/data \
    ~/Datasets/TUM_VI/dataset-room2_512_16/mav0/cam0/data.csv \
    trajectory_output.txt
```

## Usage Pattern

```bash
./mono_tum_vi <vocabulary> <settings> <image_folder_1> <times_file_1> [<image_folder_2> <times_file_2> ...] [trajectory_file]
```

### Arguments:

1. **vocabulary** - Path to ORB vocabulary file (ORBvoc.txt)
2. **settings** - YAML configuration file (TUM-VI.yaml)
3. **image_folder_N** - Path to image directory (cam0/data)
4. **times_file_N** - CSV file with timestamps (cam0/data.csv)
5. **trajectory_file** (optional) - Output trajectory filename

## Complete Example Script

Create a script to automate running:

```bash
#!/bin/bash

# Configuration
VOCAB="../../Vocabulary/ORBvoc.txt"
SETTINGS="TUM-VI.yaml"
DATASET_DIR="$HOME/Datasets/TUM_VI"
SEQUENCE="dataset-room1_512_16"

# Paths
IMAGES="$DATASET_DIR/$SEQUENCE/mav0/cam0/data"
TIMES="$DATASET_DIR/$SEQUENCE/mav0/cam0/data.csv"
OUTPUT="trajectory_room1.txt"

# Check if dataset exists
if [ ! -d "$IMAGES" ]; then
    echo "Error: Dataset not found at $IMAGES"
    echo "Please download TUM-VI dataset first"
    exit 1
fi

# Check if vocabulary exists
if [ ! -f "$VOCAB" ]; then
    echo "Error: Vocabulary file not found"
    echo "Expected: $VOCAB"
    exit 1
fi

# Run mono_tum_vi
echo "Running mono_tum_vi on $SEQUENCE..."
./mono_tum_vi "$VOCAB" "$SETTINGS" "$IMAGES" "$TIMES" "$OUTPUT"

echo "Done! Trajectory saved to $OUTPUT"
```

Save this as `run_tum_vi.sh` and make it executable:
```bash
chmod +x run_tum_vi.sh
./run_tum_vi.sh
```

## Dataset Information

### TUM-VI Sequences Overview:

| Sequence | Description | Difficulty | Duration | Size |
|----------|-------------|------------|----------|------|
| room1    | Small room, good lighting | Easy | ~2 min | ~1.5 GB |
| room2    | Medium room, textured | Medium | ~2 min | ~1.5 GB |
| room3    | Large room, challenging light | Hard | ~2 min | ~1.5 GB |
| room4    | Very large room | Hard | ~2 min | ~1.5 GB |
| room5    | Long corridor | Medium | ~2 min | ~1.5 GB |
| room6    | Large hall | Hard | ~2 min | ~1.5 GB |

### Dataset Formats Available:

- **512_16** - 512×512 resolution, 16 Hz (recommended for ORB-SLAM3)
- **512_20** - 512×512 resolution, 20 Hz
- **1024_16** - 1024×1024 resolution, 16 Hz (more detail, slower)

## Timestamp File Format

The `data.csv` file format:
```csv
#timestamp [ns],filename
1520531603419969280,1520531603419969280.png
1520531603482300928,1520531603482300928.png
...
```

## Configuration File

The `TUM-VI.yaml` file should be in the Monocular directory. Key settings:

```yaml
%YAML:1.0

# Camera Parameters
Camera.type: "PinHole"
Camera.fx: 190.97847715128717
Camera.fy: 190.9733070521226
Camera.cx: 254.93170605935475
Camera.cy: 256.8974428996504

# Camera resolution
Camera.width: 512
Camera.height: 512

# Camera frames per second
Camera.fps: 20.0

# Color order of the images (0: BGR, 1: RGB)
Camera.RGB: 1

# ORB Parameters
ORBextractor.nFeatures: 1000
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
...
```

## Expected Output

While running, you should see:
- Pangolin viewer window showing camera view and map
- Console output with tracking status
- Green points for map points
- Blue line for camera trajectory

After completion:
- `trajectory_output.txt` - Estimated camera poses
- `KeyFrameTrajectory.txt` - Keyframe trajectory (if saved)

## Troubleshooting

### "Failed to load image"
- Check image folder path is correct
- Verify images exist in the data folder
- Ensure you're pointing to `.../cam0/data` not just `.../cam0`

### "Tracking lost"
- Dataset might be too challenging - try room1 first
- Check camera calibration in TUM-VI.yaml
- Ensure good lighting in dataset

### Slow performance
- Use 512_16 resolution instead of 1024_16
- Close other applications
- Check if vocabulary file is loaded (takes ~1 sec)

## Download Commands

```bash
# Download all room sequences (512×512, 16Hz)
cd ~/Datasets/TUM_VI

# Room 1 (recommended for first test)
wget https://vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-room1_512_16.tar
tar -xf dataset-room1_512_16.tar

# Room 2
wget https://vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-room2_512_16.tar
tar -xf dataset-room2_512_16.tar

# Room 3
wget https://vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-room3_512_16.tar
tar -xf dataset-room3_512_16.tar
```

## Quick Test

```bash
# After downloading room1:
cd /home/cesar/Projects/ORB_SLAM3/Examples/Monocular

./mono_tum_vi \
    ../../Vocabulary/ORBvoc.txt \
    TUM-VI.yaml \
    $HOME/Datasets/TUM_VI/dataset-room1_512_16/mav0/cam0/data \
    $HOME/Datasets/TUM_VI/dataset-room1_512_16/mav0/cam0/data.csv \
    trajectory_room1.txt
```

You should see the Pangolin viewer open and tracking begin!

## Notes

- **First run**: Loading vocabulary takes ~1 second
- **Processing time**: Real-time or faster depending on CPU
- **GPU**: Not required but helpful for feature extraction
- **Memory**: ~2-4 GB depending on sequence length

## Alternative: TUM RGB-D Dataset

If you want to use the older TUM RGB-D dataset instead:
```bash
# Use mono_tum instead
./mono_tum \
    ../../Vocabulary/ORBvoc.txt \
    TUM1.yaml \
    ~/Datasets/rgbd_dataset_freiburg1_xyz
```

The TUM RGB-D dataset uses `rgb.txt` file format instead of CSV.
