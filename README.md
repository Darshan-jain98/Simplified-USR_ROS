# Simplified Urban Search and Rescue
The aim of this project is to simulate an urban search and rescue scenario with an explorer bot and a follower bot. The simulation is performed using ROS and gazebo.

## Prerequisites
1. Ubuntu.
2. Robot Operating System (ROS) supported by your ubuntu version.

## Setup
1. Create a catkin workspace in your home directory following this [tutorial](http://wiki.ros.org/catkin/Tutorials/create_a_workspace).
2. Install all necessary packages by executing install.bash from the repository.
3. Add these two lines to the .bashrc file located in your home directory.
```bash
export TURTLEBOT3_MODEL=waffle
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:~/catkin_ws/src/final_project/models
```

## Simulation
1. Navigate into your catkin workspace (catkin_ws, for example):
```bash
cd ~/catkin_ws
```
2. Clone this repository:
```bash
git clone https://github.com/Rutvik081/Simplified-Urban-Search-and-Rescue-ROS-Cpp.git
```
4. Build your catkin workspace:
```bash
catkin_make clean && catkin_make
```
3. Start up ROS master and ROS parameter server:
```bash
roscore
```
4.  Start Gazebo and RViz, set parameters on the Parameter Server, spawn robots, and do topic remappings:
```bash
roslaunch final_project multiple_robots.launch
```
5. Start the simplified urban search and rescue simulation:
```bash
roslaunch final_project go.launch
```
