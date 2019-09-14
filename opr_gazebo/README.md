# opr_gazebo

## Overview
* package for Gazebo simulation

## Dependency
* [mimic plugin for gazebo](https://github.com/citbrains/roboticsgroup_gazebo_plugins)
* Gazebo 7.0+
  * gazebo `version 7.0.0` has ***camera bug***
    * check gazebo version with `$ gazebo -v`

## Upgrade Gazebo
if you have gazebo version 7.0.0 run the following command to upgrade gazebo
```
$ sudo sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
$ wget http://packages.osrfoundation.org/gazebo.key
$ sudo apt-key add gazebo.key
$ sudo apt update
$ sudo apt install gazebo7
```

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
