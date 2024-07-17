

# ROS Packages for Cody Mobile Robot

## Packages

This repository contains minimal packages to control the cody robot using ROS1 noetic. 

* cody_commu: responsible for communication between ros nodes and chassis control protocols
* cody_control: some simple control cases and tools to test mobile robot performance
* cody_description: URDF model for the mobile base
* cody_msgs: cody related message definitions
  
  ## Communication interface setup
  
  Please refer to the [cody_ros/cody_commu at main · ttzntech/cody_ros · GitHub](https://github.com/ttzntech/cody_ros/tree/main/cody_commu) of "cody_commu" package for setup of communication interfaces.
  
  ## Basic usage of the ROS packages
1. Install dependent libraries
   
   ```
   $ sudo apt install -y ros-$ROS_DISTRO-teleop-twist-keyboard
   ```

2. Clone the packages into your catkin workspace and compile
    (the following instructions assume your catkin workspace is at: ~/catkin_ws/src)
   
   ```
   $ cd ~/catkin_ws/src
   $ git clone https://github.com/ttzntech/cody_ros.git
   $ cd ..
   $ catkin_make
   ```

3. Launch ROS nodes
* Start the base node for cody
  
  ```
  $ roslaunch chassis_driver cody.launch 
  ```
