cmake_minimum_required(VERSION 2.8.3)
project(pcl_process)

add_compile_options(-std=c++11)

set(OpenCV_DIR /opt/ros/kinetic/share/OpenCV-3.3.1-dev)


find_package(catkin REQUIRED COMPONENTS
        roscpp
        rospy
        std_msgs
        pcl_ros
        OpenCV REQUIRED
)

find_package(
        PCL 1.7 REQUIRED
)

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS roscpp sensor_msgs pcl_ros
)


## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
        include
  ${catkin_INCLUDE_DIRS}

)
link_directories(${PCL_LIBRARY_DIRS}
        ${OpenCV_LIBRARIES}
        )

add_executable(${PROJECT_NAME}_node src/pcl_node.cpp src/pcl_core.cpp)

target_link_libraries(${PROJECT_NAME}_node
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )
