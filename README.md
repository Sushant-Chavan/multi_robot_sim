A ROS package to simulate a fleet of robots.

# Installation

## Dependencies
1. [ROPOD sim model](https://github.com/DharminB/ropod_sim#ropod_sim)
2. [TEB local planner](http://wiki.ros.org/teb_local_planner)

## Install instructions
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

# Usage

## Single robot simulation

1. Start the default simulation configuration using the below command
    ```
    roslaunch ropod_single_robot_sim single_robot_sim.launch
    ```
2. To spawn the robot at a different pose, you can use the `init_x`, `init_y` and `init_theta` (in radians) arguments as follows:
    ```
    roslaunch ropod_single_robot_sim single_robot_sim.launch init_x:=1.0 init_y:=10.0 init_theta:=1.571
    ```
3. Several other arguments are available and can be looked up in the [launch file](ropod_single_robot_sim/ros/launch/single_robot_sim.launch).
