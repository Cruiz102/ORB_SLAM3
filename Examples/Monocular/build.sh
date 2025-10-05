#!/bin/bash

echo "Building mono_tum example..."

# Create build directory
cd "$(dirname "$0")"
mkdir -p build
cd build

# Configure and build
cmake .. -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_TYPE=ON
make -j4

echo ""
echo "Build complete!"
echo "Executable location: $(pwd)/../mono_tum"
echo ""
echo "Usage: ./mono_tum path_to_vocabulary path_to_settings path_to_sequence"
