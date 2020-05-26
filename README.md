A ROS package to simulate a fleet of robots.

## Dependencies
1. [ROPOD sim model](https://github.com/DharminB/ropod_sim#ropod_sim)
2. [TEB local planner](http://wiki.ros.org/teb_local_planner)

## Installation

1. Install dependencies
    ```
    sudo apt install -y ros-kinetic-teb-local-planner
    ```
2. Setup the catkin Workspace
    ```
    mkdir ~/catkin_ws/src/simulation
    cd ~/catkin_ws
    wstool init src/simulation
    wstool merge -t src/simulation https://raw.githubusercontent.com/Sushant-Chavan/multi-robot-sim/kinetic/multi-robot-sim.rosinstall
    wstool update -t src/simulation
    # rosdep install --from-paths src/simulation --ignore-src --rosdistro=kinetic -y
    ```
3. Build the packages
    ```
    catkin build
    ```

## Usage

TODO
