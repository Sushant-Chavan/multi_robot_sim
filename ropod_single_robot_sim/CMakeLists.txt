cmake_minimum_required(VERSION 3.0.2)
project(ropod_single_robot_sim)

find_package(catkin REQUIRED COMPONENTS
  ropod_sim
)


catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
