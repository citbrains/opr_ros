# opr_description

## Overview
* Robot URDF Model

## Usage
To view robot model on RViz
```
$ roslaunch opr_bringup display_xacro.launch model:='$(find opr_description)/robots/<robot version>' gui:=true
```
replace `<robot version>`with the following robot model

## Robot Model
* GankenKun v3 
  * `gankenkun_v3.xacro`
* GankenKun v1 (no longer maintained)
  * `gankenkun_v1.xacro`

## Package Structure
```
opr_description
|-CMakeLists.txt
|-meshes
  |-version1
    |-STL
  |-version3
    |-DAE
    |-STL
|-package.xml
|-README.md
|-robots
  |-gankenkun_v1.xacro
  |-gankenkun_v3.xacro
|-urdf
  |-gazebo.xacro
  |-materials.xacro
  |-version1
    |-arms
    |-body
    |-head
    |-legs
  |-version3
    |-arms
    |-body
    |-head
    |-legs
    |-sensors
```
