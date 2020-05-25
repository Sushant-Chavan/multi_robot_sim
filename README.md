A ROS package to simulate a fleet of robots.

## Dependencies
1. [ROPOD sim model](https://github.com/DharminB/ropod_sim#ropod_sim)

## Installation

1. Setup the catkin Workspace
    ```
    mkdir ~/catkin_ws/src/simulation
    cd ~/catkin_ws
    wstool init src/simulation
    wstool merge -t src/simulation https://raw.githubusercontent.com/Sushant-Chavan/multi-robot-sim/kinetic/multi-robot-sim.rosinstall
    wstool update -t src/simulation
    # rosdep install --from-paths src/simulation --ignore-src --rosdistro=kinetic -y
    ```
2. Build the packages
    ```
    catkin build
    ```

## Usage

TODO
