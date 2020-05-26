cmake_minimum_required(VERSION 3.0.2)
project(multi_robot_sim)

find_package(catkin REQUIRED COMPONENTS
  ropod_single_robot_sim
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
