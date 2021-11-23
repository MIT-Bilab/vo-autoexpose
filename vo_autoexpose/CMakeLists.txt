cmake_minimum_required(VERSION 3.0.2)
project(vo_autoexpose)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS 
  roscpp 
  rospy 
  std_msgs 
  message_filters 
  spinnaker_sdk_camera_driver 
  cv_bridge
  dynamic_reconfigure
)
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES mba_aeg
  CATKIN_DEPENDS roscpp rospy std_msgs message_filters spinnaker_sdk_camera_driver cv_bridge dynamic_reconfigure
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS} 
)

add_library(${PROJECT_NAME} SHARED
  src/vo_autoexpose.cpp 
  include/vo_autoexpose/vo_autoexpose.h 
  src/ 
)
target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBS} 
  ${EIGEN3_LIBS} 
)

add_executable(vo_autoexpose_node src/vo_autoexpose_node.cpp)
target_link_libraries(vo_autoexpose_node
   ${PROJECT_NAME} 
   boost_system 
   ${catkin_LIBRARIES}
)

add_executable(vo_autoexpose_stereo_node src/vo_autoexpose_stereo_node.cpp)
target_link_libraries(vo_autoexpose_stereo_node
   ${PROJECT_NAME} 
   boost_system 
   ${catkin_LIBRARIES}
)
