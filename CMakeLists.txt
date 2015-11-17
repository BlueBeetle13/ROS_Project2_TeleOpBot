cmake_minimum_required(VERSION 2.8.3)
project(project2_teleop_tank)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  genmsg
)


## Generate messages in the 'msg' folder
# add_message_files(
#  FILES
# )

## Generate services in the 'srv' folder
# add_service_files(
#   FILES
#   Service1.srv
#   Service2.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  sensor_msgs
)

## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES project2_teleop_tank
CATKIN_DEPENDS roscpp rospy std_msgs sensor_msgs message_runtime
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Tank Tracks node
add_executable(tankTracks_node src/tankTracks_node.cpp src/adafruitMotorController.cpp)
add_dependencies(tankTracks_node project2_teleop_tank_generate_messages_cpp)
target_link_libraries(tankTracks_node
  ${catkin_LIBRARIES}
  wiringPi
  wiringPiDev
)

# Camera Pan-Tilt node
add_executable(cameraPanTilt_node src/cameraPanTilt_node.cpp src/adafruitServoController.cpp)
add_dependencies(cameraPanTilt_node project2_teleop_tank_generate_messages_cpp)
target_link_libraries(cameraPanTilt_node
  ${catkin_LIBRARIES}
  wiringPi
  wiringPiDev
)
