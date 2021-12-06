cmake_minimum_required(VERSION 3.0.2)
project(sv_slam)

add_compile_options(-std=c++14)
set(CMAKE_BUILD_TYPE "Release")
# set(CMAKE_BUILD_TYPE "Debug")


# Find catkin macros and libraries if COMPONENTS list like find_package(catkin
# REQUIRED COMPONENTS xyz) is used, also find other catkin packages
find_package(
        catkin REQUIRED
        COMPONENTS roscpp
        std_msgs
        sensor_msgs
        geometry_msgs
        genmsg
        cv_bridge
)

find_package(OpenCV REQUIRED)
find_package(Eigen3)


find_package(PCL REQUIRED)


# 指定libTorch位置
set(Torch_DIR /home/chrisliu/ROSws/SV-SLAM_ws/src/SV-SLAM/lib/libtorch/share/cmake/Torch)
find_package(Torch REQUIRED)
message(STATUS "Torch library status:")
message(STATUS "    version: ${TORCH_VERSION}")
message(STATUS "    libraries: ${TORCH_LIBS}")
message(STATUS "    include path: ${TORCH_INCLUDE_DIRS}")
message(STATUS "    torch lib : ${TORCH_LIBRARIES} ")



include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR}
        ${PCL_INCLUDE_DIRS}
        )

catkin_package(
        # INCLUDE_DIRS include LIBRARIES mmdetection_ros CATKIN_DEPENDS
        # other_catkin_pkg DEPENDS system_lib
)

find_package(CUDA)

include_directories(
        lib/Line3Dpp
        lib/Line3Dpp/build
        ${CUDA_INCLUDE_DIRS}
        )
link_directories(
        lib/Line3Dpp/build)

add_library(segmentor_lib SHARED
        include/sensor/camera/segmentor/segmentor.cpp)
target_link_libraries(segmentor_lib ${catkin_LIBRARIES}
        ${OpenCV_LIBS} dw 
        ${TORCH_LIBRARIES}
        )

add_library(
        sensor_lib SHARED
        include/sensor/sensor.cpp include/sensor/gps/gps.cpp
        include/sensor/camera/camera.cpp include/sensor/lidar/lidar.cpp)
target_link_libraries(sensor_lib ${catkin_LIBRARIES} ${OpenCV_LIBS}
        ${PCL_LIBRARIES} segmentor_lib)


add_library(
        map_lib SHARED
        include/map/map.cpp)
target_link_libraries(map_lib ${catkin_LIBRARIES} ${OpenCV_LIBS}
        ${PCL_LIBRARIES}
        line3Dpp
        )

add_library(
        sv_slam_lib SHARED
        include/sv_slam.cpp)
target_link_libraries(sv_slam_lib ${catkin_LIBRARIES} ${OpenCV_LIBS}
        ${PCL_LIBRARIES} sensor_lib 
        # map_lib
        )

add_executable(sv_slam_node src/sv_slam_node.cpp)
target_link_libraries(sv_slam_node sv_slam_lib dw)
