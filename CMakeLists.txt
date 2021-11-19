cmake_minimum_required(VERSION 3.0.2)
project(sv_slam)

add_compile_options(-std=c++14)
# set(CMAKE_BUILD_TYPE "Release")

# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
  catkin REQUIRED
  COMPONENTS roscpp
             std_msgs
             # actionlib cv_bridge image_transport
             sensor_msgs
             # control_msgs
             geometry_msgs
             genmsg
             # actionlib_msgs set_nav_goal vision_msgs
             cv_bridge
             )

find_package(OpenCV REQUIRED)
find_package(Eigen3)
find_package(PCL REQUIRED)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
                    ${PCL_INCLUDE_DIRS})

catkin_package(
  # INCLUDE_DIRS include LIBRARIES mmdetection_ros CATKIN_DEPENDS
  # other_catkin_pkg DEPENDS system_lib
)

# # https://pybind11.readthedocs.io/en/stable/compiling.html#building-with-cmake
set(Python3_ROOT_DIR "/home/chrisliu/anaconda3/envs/open-mmlab")
find_package(
  Python3
  COMPONENTS Interpreter Development
  REQUIRED)

catkin_python_setup()

add_subdirectory(lib/pybind11)

# https://pybind11.readthedocs.io/en/stable/advanced/embedding.html
add_library(segmentor_lib SHARED include/sensor/image/segmentor/segmentor.h
                                 include/sensor/image/segmentor/segmentor.cpp)
target_link_libraries(segmentor_lib pybind11::embed ${catkin_LIBRARIES}
                      ${OpenCV_LIBS})

add_library(
  sv_slam_lib SHARED
  include/sv_slam.cpp include/sensor/sensor.cpp include/sensor/gps/gps.cpp
  include/sensor/image/image.cpp include/sensor/lidar/lidar.cpp)
target_link_libraries(sv_slam_lib ${catkin_LIBRARIES} ${OpenCV_LIBS}
                      ${PCL_LIBRARIES} segmentor_lib dw)

add_executable(sv_slam_node src/sv_slam_node.cpp)
target_link_libraries(sv_slam_node sv_slam_lib)
