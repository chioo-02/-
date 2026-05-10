cmake_minimum_required(VERSION 3.0.2)
project(competition_bringup)

find_package(catkin REQUIRED COMPONENTS roscpp yaml-cpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES competition_bringup
  CATKIN_DEPENDS roscpp
)

include_directories(include ${catkin_INCLUDE_DIRS})

# Map utilities library
add_library(competition_bringup
  src/map_utils.cpp
)

target_link_libraries(competition_bringup ${catkin_LIBRARIES} ${yaml-cpp_LIBRARIES})

install(TARGETS competition_bringup
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(DIRECTORY config/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
