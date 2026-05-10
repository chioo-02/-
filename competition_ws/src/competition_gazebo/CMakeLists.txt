cmake_minimum_required(VERSION 3.0.2)
project(competition_gazebo)

find_package(catkin REQUIRED)

catkin_package()

install(DIRECTORY worlds models launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
