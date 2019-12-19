# opr_ros
ROS package for CIT Open Platform Robot GankenKun. Tested on ROS Kinetic

## Installation
1. Install ros
2. create ROS workspace
```
$ sudo apt install python-catkin-tools
$ mkdir -p ~/catkin_ws/src
$ cd ~/catkin_ws
$ catkin init
$ source `catkin locate --shell-verbs`
```
3. clone repository
```
$ cd ~/catkin_ws/src
$ git clone https://github.com/citbrains/opr_ros.git
$ wstool init .
$ wstool merge opr_ros/opr_ros.rosinstall
$ wstool update
$ rosdep install --from-paths opr_ros --ignore-src --rosdistro kinetic
```
4. build and source
```
$ cd ~/catkin_ws
$ catkin build opr_ros
$ catkin source
```

## Usage
* View URDF model on RViz
```
$ roslaunch opr_bringup display_xacro.launch model:='$(find opr_description)/robots/gankenkun_v3.xacro' gui:=true
```
* Gazebo Simulation
```
$ roslaunch opr_bringup simulation.launch
```

## Package Description
* opr_bringup
  * collection of launch files to bring up actual robot and robot simulation
* opr_description
  * collection of robot model's urdf, CAD data (stl), collada data (dae)
* opr_gazebo
  * collection of gazebo related files
* opr_kondo_driver
  * collection of B3M servo motor related files
* opr_ros
  * package to build other package
* opr_ros_control
  * collection of ros controllers
