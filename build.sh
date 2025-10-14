echo "Configuring and building Thirdparty/DBoW2 ..."

cd Thirdparty/DBoW2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
make install

cd ../../g2o

echo "Configuring and building Thirdparty/g2o ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
sudo make install

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j
sudo make install

cd ../../../

echo "Uncompress vocabulary ..."

cd Vocabulary
tar -xf ORBvoc.txt.tar.gz
cd ..

echo "Configuring and building ORB_SLAM3 ..."


mkdir build
cd build
# Source ROS2 environment and set include paths for Pangolin
source /opt/ros/jazzy/setup.bash
cmake .. -DCMAKE_BUILD_TYPE=Debug \
         -DCMAKE_EXPORT_TYPE=ON \
         -DCMAKE_PREFIX_PATH="/opt/ros/jazzy" \
         -DCMAKE_CXX_FLAGS="-I/opt/ros/jazzy/include"
make -j4
sudo make install
