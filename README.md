A ROS package to simulate a fleet of robots.

## Installation

1. Setup the catkin Workspace
    ```
    mkdir ~/catkin_ws/src/multi_robot_sim
    cd ~/catkin_ws
    wstool init src/multi_robot_sim
    wstool merge -t src/multi_robot_sim https://raw.githubusercontent.com/Sushant-Chavan/multi-robot-sim/kinetic/multi-robot-sim.rosinstall
    wstool update -t src/multi_robot_sim
    # rosdep install --from-paths src/multi_robot_sim --ignore-src --rosdistro=kinetic -y
    ```
2. Build the packages
    ```
    catkin build
    ```

## Usage

TODO
