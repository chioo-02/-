cmake_minimum_required(VERSION 3.0.2)
project(mission_node)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
  mavros_msgs
  nav_msgs
  std_srvs
)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(mission_node
  src/mission_node.cpp
)

target_link_libraries(mission_node ${catkin_LIBRARIES})

install(TARGETS mission_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
