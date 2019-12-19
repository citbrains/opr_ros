# opr_bringup

## Overview
* package to launch simulator

## Usage
* To view robot model on RViz
```
$ roslaunch opr_bringup display_xacro.launch model:='$(find opr_description)/robots/gankenkun_v3.xacro' gui:=true
```
* To simulate robot on gazebo ***recommended to view with GPU***
```
$ roslaunch opr_bringup simulation.launch
```
## Package Structure
```
opr_bringup
|-CMakeLists.txt
|-launch/
  |-display_xacro.launch
|-package.xml
|-README.md
```


