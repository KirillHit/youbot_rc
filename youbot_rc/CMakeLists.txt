cmake_minimum_required(VERSION 3.0.2)
project(youbot_rc)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  #brics_actuator
  geometry_msgs
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
#  LIBRARIES youbot_rc
#  CATKIN_DEPENDS brics_actuator geometry_msgs roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_subdirectory(lib)

add_executable(youbot_rc_node src/youbot_rc_node.cpp)

add_dependencies(youbot_rc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(youbot_rc_node
  ${catkin_LIBRARIES}
  simple_socket
)



