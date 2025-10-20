cd build
# Source ROS2 environment and set include paths for Pangolin
source /opt/ros/jazzy/setup.bash
cmake .. -DCMAKE_BUILD_TYPE=Debug \
         -DCMAKE_EXPORT_TYPE=ON \
         -DCMAKE_PREFIX_PATH="/opt/ros/jazzy" \
         -DCMAKE_CXX_FLAGS="-I/opt/ros/jazzy/include"
make -j4
sudo make install
