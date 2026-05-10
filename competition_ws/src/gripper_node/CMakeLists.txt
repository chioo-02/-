cmake_minimum_required(VERSION 3.0.2)
project(gripper_node)

find_package(catkin REQUIRED COMPONENTS roscpp std_msgs)

catkin_package()

include_directories(include ${catkin_INCLUDE_DIRS})

add_executable(gripper_node src/gripper_node.cpp)

target_link_libraries(gripper_node ${catkin_LIBRARIES})

install(TARGETS gripper_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
