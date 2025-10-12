cmake_minimum_required(VERSION 2.8)
project(DBoW2)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS}  -Wall  -O3 -march=native ")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall  -O3 -march=native")

set(HDRS_DBOW2
  DBoW2/BowVector.h
  DBoW2/FORB.h 
  DBoW2/FClass.h       
  DBoW2/FeatureVector.h
  DBoW2/ScoringObject.h   
  DBoW2/TemplatedVocabulary.h)
set(SRCS_DBOW2
  DBoW2/BowVector.cpp
  DBoW2/FORB.cpp      
  DBoW2/FeatureVector.cpp
  DBoW2/ScoringObject.cpp)

set(HDRS_DUTILS
  DUtils/Random.h
  DUtils/Timestamp.h)
set(SRCS_DUTILS
  DUtils/Random.cpp
  DUtils/Timestamp.cpp)


find_package(OpenCV 4 QUIET)
if(NOT OpenCV_FOUND)
   find_package(OpenCV 3.0 QUIET)
   if(NOT OpenCV_FOUND)
      message(FATAL_ERROR "OpenCV > 3.0 not found.")
   endif()
endif()

set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(${OpenCV_INCLUDE_DIRS})
add_library(DBoW2 SHARED ${SRCS_DBOW2} ${SRCS_DUTILS})
target_link_libraries(DBoW2 ${OpenCV_LIBS})

# Include GNUInstallDirs for standard installation paths
include(GNUInstallDirs)

# Install the DBoW2 library
install(TARGETS DBoW2
        EXPORT DBoW2Targets
        LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
        ARCHIVE DESTINATION ${CMAKE_INSTALL_LIBDIR}
        RUNTIME DESTINATION ${CMAKE_INSTALL_BINDIR}
        INCLUDES DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})

# Install header files
install(FILES ${HDRS_DBOW2}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/DBoW2)

install(FILES ${HDRS_DUTILS}
        DESTINATION ${CMAKE_INSTALL_INCLUDEDIR}/DUtils)

# Export targets for other projects to use
install(EXPORT DBoW2Targets
        FILE DBoW2Targets.cmake
        NAMESPACE DBoW2::
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/DBoW2)

# Create and install package config files
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    DBoW2ConfigVersion.cmake
    VERSION 1.0.0
    COMPATIBILITY SameMajorVersion)

configure_package_config_file(
    "${CMAKE_CURRENT_SOURCE_DIR}/DBoW2Config.cmake.in"
    "${CMAKE_CURRENT_BINARY_DIR}/DBoW2Config.cmake"
    INSTALL_DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/DBoW2)

install(FILES "${CMAKE_CURRENT_BINARY_DIR}/DBoW2Config.cmake"
              "${CMAKE_CURRENT_BINARY_DIR}/DBoW2ConfigVersion.cmake"
        DESTINATION ${CMAKE_INSTALL_LIBDIR}/cmake/DBoW2)

