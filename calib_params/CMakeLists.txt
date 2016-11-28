cmake_minimum_required(VERSION 2.8.3)
project(calib_params)

find_package(cmake_modules REQUIRED)


## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  cmake_modules
  message_generation
  )

## System dependencies are found with CMake's conventions
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(OpenCV 2 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

################################################
## Declare ROS messages, services and actions ##
################################################

## To declare and build messages, services or actions from within this
## package, follow these steps:
## * Let MSG_DEP_SET be the set of packages whose message types you use in
##   your messages/services/actions (e.g. std_msgs, actionlib_msgs, ...).
## * In the file package.xml:
##   * add a build_depend tag for "message_generation"
##   * add a build_depend and a run_depend tag for each package in MSG_DEP_SET
##   * If MSG_DEP_SET isn't empty the following dependency has been pulled in
##     but can be declared for certainty nonetheless:
##     * add a run_depend tag for "message_runtime"
## * In this file (CMakeLists.txt):
##   * add "message_generation" and every package in MSG_DEP_SET to
##     find_package(catkin REQUIRED COMPONENTS ...)
##   * add "message_runtime" and every package in MSG_DEP_SET to
##     catkin_package(CATKIN_DEPENDS ...)
##   * uncomment the add_*_files sections below as needed
##     and list every .msg/.srv/.action file to be processed
##   * uncomment the generate_messages entry below
##   * add every package in MSG_DEP_SET to generate_messages(DEPENDENCIES ...)

## Generate messages in the 'msg' folder
add_message_files(
  DIRECTORY msg
  FILES
  CameraCalibMsg.msg  
  LidarCalibMsg.msg  
  RGBDCalibMsg.msg
)

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate actions in the 'action' folder
# add_action_files(
#   FILES
#   Action1.action
#   Action2.action
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
    DEPENDENCIES
    #geometry_msgs#   nav_msgs#   sensor_msgs#   
    std_msgs#   visualization_msgs
)

catkin_package(
  INCLUDE_DIRS include/
  LIBRARIES
  CATKIN_DEPENDS cv_bridge roscpp rospy std_msgs message_runtime 
  DEPENDS Eigen 
  )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
# include_directories(include)
include_directories(
  include/calib_params
  include           
  ${catkin_INCLUDE_DIRS}
  ${INCLUDE_DIRS})


## Declare a C++ library

add_library(${PROJECT_NAME}	src/camera_calib_params.cc
										src/lidar_calib_params.cc  
										src/rgbd_calib_params.cc
)
target_link_libraries(${PROJECT_NAME} yaml-cpp)
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})
