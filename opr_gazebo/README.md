# opr_gazebo

## Overview
* package for Gazebo simulation

## Dependency
* [mimic plugin for gazebo](https://github.com/citbrains/roboticsgroup_gazebo_plugins)

## Usage
* view empty world on gazebo with robot model
```
$ roslaunch opr_gazebo gazebo.launch model:='$(find opr_description)/robots/gankenkun_v3.xacro' 
```

## Package Structure
```
opr_gazebo
|-CMakeLists.txt
|-launch
  |-gazebo.launch
|-package.xml
|-README.md
```
