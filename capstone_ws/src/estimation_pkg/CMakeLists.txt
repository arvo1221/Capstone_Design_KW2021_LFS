cmake_minimum_required(VERSION 3.0.2)
project(estimation_pkg)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  geometry_msgs
  image_transport
  message_filters
  message_generation
  pcl_conversions
  pcl_ros
  roscpp
  rospy
  sensor_msgs
  std_msgs
  gb_visual_detection_3d_msgs
  tf
  serial
  turret_assembly
)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(PCL 1.7 REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES estimation_pkg
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_filters pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs gb_visual_detection_3d_msgs
  DEPENDS system_lib
  DEPENDS Eigen3
)

# set link between hpp and cpp 
set(SRCS
  src/turret_controller.cpp
  src/drone_estimator.cpp
  src/serialComm/cserial.cpp
  src/serialComm/serial_turret.cpp
)
set(HEADS
  include/estimation_pkg/turret_controller.hpp
  include/estimation_pkg/drone_estimator.hpp
  include/estimation_pkg/serialComm/DataType.h
  include/estimation_pkg/serialComm/cserial.h
  include/estimation_pkg/serialComm/serial_turret.hpp
)
add_library(turret_controller ${SRCS} ${HEADS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

#target_link_libraries(${PROJECT_NAME}_node
#  ${catkin_LIBRARIES}
##  ${Boost_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
