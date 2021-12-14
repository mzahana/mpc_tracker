cmake_minimum_required(VERSION 3.0.2)
project(mpc_tracker)

set (CMAKE_CXX_STANDARD 11)


find_package(Eigen3)
find_package(OsqpEigen)
find_package(geodetic_utils REQUIRED)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  sensor_msgs
  nav_msgs
  nlopt
  OsqpEigen
  osqp
  message_generation
  plotty
  mav_msgs
  mav_trajectory_generation
  mav_trajectory_generation_ros
  geodetic_utils
  trajectory_msgs
)


add_message_files(
  DIRECTORY msg
  FILES
  State.msg
  StateTrajectory.msg
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  ExecutePathFromFile.srv
  GoToHeight.srv
  GoToPoseWaypoints.srv
  GoToWaypoint.srv
  GoToWaypoints.srv
)

generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs sensor_msgs message_runtime trajectory_msgs
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIR}
  #${Eigen_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)

#
# mpc_tracker
#
add_library(mpc_tracker
  src/mpc_tracker.cpp 
)
add_dependencies(mpc_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_tracker ${catkin_LIBRARIES} osqp::osqp)

# MPC tracker
add_executable(mpc_tracker_node
  src/mpc_tracker_node.cpp
)
add_dependencies(mpc_tracker_node mpc_tracker ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mpc_tracker_node mpc_tracker ${catkin_LIBRARIES} OsqpEigen::OsqpEigen osqp::osqp)

#
# waypoint_navigator
#
add_executable(navigator_node
  src/navigator_node.cpp
)
add_dependencies(navigator_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(navigator_node ${catkin_LIBRARIES})