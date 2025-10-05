cmake_minimum_required(VERSION 2.8)
project(mono_tum)

# Generate compile_commands.json for IDE support
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Set C++14 standard
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

IF(NOT CMAKE_BUILD_TYPE)
  SET(CMAKE_BUILD_TYPE Release)
ENDIF()

MESSAGE("Build type: " ${CMAKE_BUILD_TYPE})

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -Wall -O3")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -O3")
set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -march=native")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -march=native")

# Suppress specific warnings
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated-declarations")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-reorder")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-sign-compare")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-unused-variable")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wno-deprecated")

LIST(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/../../cmake_modules)

# Find required packages
find_package(OpenCV 4.4 REQUIRED)
if(NOT OpenCV_FOUND)
   message(FATAL_ERROR "OpenCV > 4.4 not found.")
endif()

find_package(Eigen3 3.1.0 REQUIRED)
find_package(Pangolin REQUIRED)

# Include directories
include_directories(
${PROJECT_SOURCE_DIR}
${PROJECT_SOURCE_DIR}/../../
${PROJECT_SOURCE_DIR}/../../include
${PROJECT_SOURCE_DIR}/../../include/CameraModels
${PROJECT_SOURCE_DIR}/../../Thirdparty/Sophus
${EIGEN3_INCLUDE_DIR}
)

# Set the path to ORB_SLAM3 library
set(ORB_SLAM3_LIB_DIR ${PROJECT_SOURCE_DIR}/../../lib)
set(ORB_SLAM3_LIBRARY ${ORB_SLAM3_LIB_DIR}/libORB_SLAM3.so)

# Check if ORB_SLAM3 library exists
if(NOT EXISTS ${ORB_SLAM3_LIBRARY})
    message(FATAL_ERROR "ORB_SLAM3 library not found at ${ORB_SLAM3_LIBRARY}. Please build the main library first.")
endif()

# Build mono_tum executable
add_executable(mono_tum
mono_tum.cc)

target_link_libraries(mono_tum
${ORB_SLAM3_LIBRARY}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
-lboost_serialization
-lcrypto
)

# Set output directory
set_target_properties(mono_tum PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}
)

# Build mono_tum_vi executable
add_executable(mono_tum_vi
mono_tum_vi.cc)

target_link_libraries(mono_tum_vi
${ORB_SLAM3_LIBRARY}
${OpenCV_LIBS}
${EIGEN3_LIBS}
${Pangolin_LIBRARIES}
-lboost_serialization
-lcrypto
)

# Set output directory
set_target_properties(mono_tum_vi PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}
)

message(STATUS "Configured mono_tum and mono_tum_vi examples")
message(STATUS "OpenCV version: ${OpenCV_VERSION}")
message(STATUS "ORB_SLAM3 library: ${ORB_SLAM3_LIBRARY}")
