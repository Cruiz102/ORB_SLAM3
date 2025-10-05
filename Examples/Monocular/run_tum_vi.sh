#!/bin/bash

# TUM-VI Dataset Runner Script
# This script helps you run mono_tum_vi with proper arguments

# Color codes
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m' # No Color

echo -e "${BLUE}========================================${NC}"
echo -e "${BLUE}    TUM-VI Dataset Runner${NC}"
echo -e "${BLUE}========================================${NC}"
echo ""

# Configuration
VOCAB="../../Vocabulary/ORBvoc.txt"
SETTINGS="TUM-VI.yaml"
DATASET_DIR="${HOME}/Datasets/TUM_VI"

# Check if mono_tum_vi exists
if [ ! -f "./mono_tum_vi" ]; then
    echo -e "${RED}❌ Error: mono_tum_vi not found${NC}"
    echo "Please build it first with: ./build.sh"
    exit 1
fi

# Check if vocabulary exists
if [ ! -f "$VOCAB" ]; then
    echo -e "${RED}❌ Error: Vocabulary file not found at: $VOCAB${NC}"
    exit 1
fi

# Check if settings file exists
if [ ! -f "$SETTINGS" ]; then
    echo -e "${RED}❌ Error: Settings file not found: $SETTINGS${NC}"
    exit 1
fi

# Show usage if no arguments
if [ $# -eq 0 ]; then
    echo -e "${YELLOW}Usage:${NC}"
    echo "  $0 <sequence_name> [output_trajectory.txt]"
    echo ""
    echo -e "${YELLOW}Examples:${NC}"
    echo "  $0 room1"
    echo "  $0 room1 my_trajectory.txt"
    echo "  $0 dataset-room1_512_16"
    echo ""
    echo -e "${YELLOW}Available sequences:${NC}"
    
    # List available sequences
    if [ -d "$DATASET_DIR" ]; then
        echo -e "${GREEN}Found in $DATASET_DIR:${NC}"
        for seq in "$DATASET_DIR"/dataset-*; do
            if [ -d "$seq" ]; then
                basename "$seq"
            fi
        done
    else
        echo -e "${RED}Dataset directory not found: $DATASET_DIR${NC}"
        echo ""
        echo "To download TUM-VI dataset:"
        echo "  mkdir -p $DATASET_DIR"
        echo "  cd $DATASET_DIR"
        echo "  wget https://vision.in.tum.de/tumvi/exported/euroc/512_16/dataset-room1_512_16.tar"
        echo "  tar -xf dataset-room1_512_16.tar"
    fi
    echo ""
    echo "For more information, see: RUNNING_TUM_VI.md"
    exit 0
fi

# Parse sequence name
SEQUENCE_NAME="$1"
OUTPUT_TRAJECTORY="${2:-trajectory_${SEQUENCE_NAME}.txt}"

# Handle short names (room1 -> dataset-room1_512_16)
if [[ "$SEQUENCE_NAME" == room* ]] && [[ ! "$SEQUENCE_NAME" == dataset-* ]]; then
    SEQUENCE_NAME="dataset-${SEQUENCE_NAME}_512_16"
fi

# Build paths
SEQUENCE_PATH="$DATASET_DIR/$SEQUENCE_NAME"
IMAGES="$SEQUENCE_PATH/mav0/cam0/data"
TIMES="$SEQUENCE_PATH/mav0/cam0/data.csv"

echo -e "${BLUE}Configuration:${NC}"
echo "  Vocabulary: $VOCAB"
echo "  Settings:   $SETTINGS"
echo "  Sequence:   $SEQUENCE_NAME"
echo "  Images:     $IMAGES"
echo "  Times:      $TIMES"
echo "  Output:     $OUTPUT_TRAJECTORY"
echo ""

# Validate paths
errors=0

if [ ! -d "$SEQUENCE_PATH" ]; then
    echo -e "${RED}❌ Sequence not found: $SEQUENCE_PATH${NC}"
    errors=$((errors + 1))
fi

if [ ! -d "$IMAGES" ]; then
    echo -e "${RED}❌ Image directory not found: $IMAGES${NC}"
    errors=$((errors + 1))
fi

if [ ! -f "$TIMES" ]; then
    echo -e "${RED}❌ Timestamp file not found: $TIMES${NC}"
    errors=$((errors + 1))
fi

if [ $errors -gt 0 ]; then
    echo ""
    echo -e "${YELLOW}💡 To download this sequence:${NC}"
    echo "  cd $DATASET_DIR"
    echo "  wget https://vision.in.tum.de/tumvi/exported/euroc/512_16/${SEQUENCE_NAME}.tar"
    echo "  tar -xf ${SEQUENCE_NAME}.tar"
    exit 1
fi

# Count images
num_images=$(ls -1 "$IMAGES"/*.png 2>/dev/null | wc -l)
echo -e "${GREEN}✅ Found $num_images images${NC}"
echo ""

# Show estimated processing time
echo -e "${YELLOW}⏱️  Loading vocabulary (this may take a few seconds)...${NC}"
echo ""

# Run mono_tum_vi
echo -e "${GREEN}🚀 Starting mono_tum_vi...${NC}"
echo ""
echo "Press Ctrl+C to stop"
echo "----------------------------------------"
echo ""

./mono_tum_vi "$VOCAB" "$SETTINGS" "$IMAGES" "$TIMES" "$OUTPUT_TRAJECTORY"

# Check result
if [ $? -eq 0 ]; then
    echo ""
    echo "----------------------------------------"
    echo -e "${GREEN}✅ Processing completed successfully!${NC}"
    
    if [ -f "$OUTPUT_TRAJECTORY" ]; then
        num_poses=$(wc -l < "$OUTPUT_TRAJECTORY")
        echo -e "${GREEN}📝 Trajectory saved: $OUTPUT_TRAJECTORY ($num_poses poses)${NC}"
    fi
    
    if [ -f "KeyFrameTrajectory.txt" ]; then
        num_kf=$(wc -l < "KeyFrameTrajectory.txt")
        echo -e "${GREEN}📝 KeyFrame trajectory: KeyFrameTrajectory.txt ($num_kf keyframes)${NC}"
    fi
else
    echo ""
    echo -e "${RED}❌ Processing failed or was interrupted${NC}"
    exit 1
fi

echo ""
echo -e "${BLUE}Done!${NC}"
