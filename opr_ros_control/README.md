# opr_ros_control

## Overview
* package to launch ros controllers

## Dependency

## Usage
***Recommended to load hardware interface or gazebo simulation***
* gazebo control ***only for gazebo simulation with position or trajectory position control***
```
$ roslaunch opr_ros_control gazebo_controller.launch
```

* position control
```
$ roslaunch opr_ros_control position_controller.launch
```

* trajectory postion control
```
$ roslaunch opr_ros_control trajectory_position_controller.launch
```

## Package Structure
```
opr_ros_control
|-CMakeLists.txt
|-config/
  |-effort_controller.yaml
  |-gazebo_controller.yaml
  |-joint_state_controller.yaml
  |-position_controller.yaml
  |-trajectory_effort_controller.yaml
  |-trajectory_position_controller.yaml
|-launch/
  |-gazebo_controller.launch
  |-position_controller.launch
  |-trajectory_position_controller.launch
|-package.xml
|-README.md
```
