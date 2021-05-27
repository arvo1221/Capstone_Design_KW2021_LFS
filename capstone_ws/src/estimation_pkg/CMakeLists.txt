cmake_minimum_required(VERSION 3.0.2)
project(estimation_pkg)
add_compile_options(-std=c++11 -pthread )

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
  #trajectory_msgs
  kdl_parser # IK
  trac_ik_lib  #track-IK
  pcl_conversions  #PCL INCLUDES
)

# RBDL setting
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../rbdl" )
LIST( APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/CMake" )
INCLUDE(CMake/FindRBDL.cmake)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Boost REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(RBDL REQUIRED)
#find_package(Boost REQUIRED COMPONENTS date_time)   # track-IK
find_package(orocos_kdl REQUIRED)                   # track-IK

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES estimation_pkg
  CATKIN_DEPENDS cv_bridge geometry_msgs image_transport message_filters pcl_conversions pcl_ros roscpp rospy sensor_msgs std_msgs gb_visual_detection_3d_msgs  kdl_parser urdf
  DEPENDS system_lib Boost orocos_kdl
  DEPENDS Eigen3
)

# set link between hpp and cpp 

link_directories(${Boost_LIBRARY_DIR} ${orocos_kdl_LIBRARIES} ${PCL_LIBRARY_DIRS} ${OpenCV_LIBRARY_DIRS})

set(SRCS
  src/turret_controller.cpp
  src/drone_estimator.cpp
  src/Kinematics.cpp
  src/serialComm/cserial.cpp
  src/serialComm/cserial2.cpp
  #src/serialComm/Comm.cpp
  src/serialComm/t_serial.cpp
  src/serialComm/serial_turret.cpp
  #src/main.cpp
)
set(HEADS
  include/estimation_pkg/turret_controller.hpp
  include/estimation_pkg/drone_estimator.hpp
  include/estimation_pkg/YamlConfig.h
  include/estimation_pkg/Kinematics.h
  include/estimation_pkg/serialComm/DataType.h
  include/estimation_pkg/serialComm/DataType2.h
  include/estimation_pkg/serialComm/cserial.h
  include/estimation_pkg/serialComm/cserial2.h
  include/estimation_pkg/serialComm/t_serial.h
  #include/estimation_pkg/serialComm/Comm.h
  include/estimation_pkg/serialComm/serial_turret.hpp
)
add_library(turret_controller ${SRCS} ${HEADS})
#target_link_libraries(turret_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES} rbdl rbdl_urdfreader yaml-cpp trajectory)
target_link_libraries(turret_controller ${catkin_LIBRARIES} ${Boost_LIBRARIES} ${orocos_kdl_INCLUDE_DIRS} rbdl rbdl_urdfreader yaml-cpp)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  ${RBDL_INCLUDE_DIRS}
  ${orocos_kdl_INCLUDE_DIRS}
)
#add_library(CommandCenter ${SRCS} ${HEADS})
add_executable(CommandCenter src/main.cpp ${SRCS} ${HEADS})
target_link_libraries(CommandCenter turret_controller)

#target_link_libraries(${PROJECT_NAME}_node
#  ${catkin_LIBRARIES}
##  ${Boost_LIBRARIES}
#  ${PCL_LIBRARIES}
#)
