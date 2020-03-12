# opr_user_interface

## Overview
* collection of GUI and custom RViz Panel to interface with the Open Platform Robot

## Usage
* GUI only
```
$ rosrun opr_user_interface statusUI
```

* RViz Interface
1. Start Rviz
2. `Panels` -> `Add New Panel` -> `opr_user_interface` -> `HuSendCom`

## Package Structure
```
|-opr_user_interface
|-CMakeLists.txt
|-include/
  |-statusui.hpp
  |-husendcom.hpp
  |-panel.hpp
|-package.xml
|-plugin_description.xml
|=README.md
|-src/
  |-statusui.cpp
  |-husendcom.cpp
  |-panel.cpp
```
