# opr_gazebo

## Overview
* package for Gazebo simulation

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
