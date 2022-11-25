cmake_minimum_required(VERSION 3.0.2)
project(pos_controller)

find_package(catkin REQUIRED COMPONENTS
  mavros
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
)

add_service_files(
        FILES
        pid.srv
)

generate_messages(
        DEPENDENCIES
        std_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)


###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/pos_controller
  ${catkin_INCLUDE_DIRS}
)

add_executable(pos_controller src/pos_controller_node.cpp src/PID.cpp)
target_link_libraries(pos_controller ${catkin_LIBRARIES})
#add_dependencies(pos_controller abc)