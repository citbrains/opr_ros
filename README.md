# opr_ros
ROS package for CIT Open Platform Robot GankenKun. Tested on ROS Kinetic

## Installation
1. Install ros
2. create ROS workspace
```
$ mkdir -p ~/catkin_ws/src
```
3. clone repository
```
$ cd ~/catkin_ws/src
$ git clone git@github.com:citbrains/opr_ros.git
$ wstool init .
$ wstool merge opr_ros/opr_ros.rosinstall
$ wstool update
$ rosdep install --from-paths opr_ros --ignore-src --rosdistro kinetic
```
4. build and source
```
$ cd ~/catkin_ws
$ catkin build opr_ros
$ source ~/catkin_ws/devel/setup.bash
```

## Usage
To view URDF model on RViz
```
roslaunch opr_bringup display_xacro.launch model:='$(find opr_description)/robots/gankenkun_v3.xacro' gui:=true
```

## Package Description
* opr_bringup
  * collection of launch files to bring up actual robot and robot simulation
* opr_description
  * collection of robot model's urdf, CAD data (stl), collada data (dae)
* opr_gazebo
  * collection of gazebo related files
* opr_ros
  * package to build other package
* opr_ros_control
  * collection of ros controllers
