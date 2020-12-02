cmake_minimum_required(VERSION 2.8.3)
project(topic_bridge)

set( CMAKE_BUILD_TYPE Release )
# set( CMAKE_BUILD_TYPE Debug )

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O2 -m64 -flto -ffast-math -msse3 -march=native -mfpmath=sse")
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O0")

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  cv_bridge
  sensor_msgs
)

find_package(OpenCV REQUIRED)

catkin_package()

message(STATUS "OpenCV library status:")
message(STATUS "  version: ${OpenCV_VERSION}")
message(STATUS "  libraries: ${OpenCV_LIBS}")
message(STATUS "  include path: ${OpenCV_INCLUDE_DIRS}")

###########
## Build ##
###########
include_directories(
  ${PROJECT_SOURCE_DIR}
  ${PROJECT_SOURCE_DIR}/include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(image_bridge src/image_bridge.cpp)
target_link_libraries(image_bridge
  ${catkin_LIBRARIES}
  ${OpenCV_LIBS}
)

add_executable(imu_bridge src/imu_bridge.cpp)
target_link_libraries(imu_bridge
  ${catkin_LIBRARIES}
)
