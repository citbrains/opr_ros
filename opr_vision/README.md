# opr_vision

## Overview
* opr vision related ros package

## Dependency
* [uvc_camera](http://wiki.ros.org/uvc_camera)
* [darknet_ros](https://github.com/citbrains/darknet_ros)

## ROS API
### Subscribed topics
* /image_raw
  * Topic to receive raw images
### Published topics (TODO)
* /brain/ballpos
  * Topic to publish ball postion
  * Topic type: `opr_msgs/ballpos`
    * geometry_msgs/Pose2D `global`: global coordinate
    * geometry_msgs/Pose2D `local`: local coordinate
* /vision/whitelines
  * Topic to publish white lines

### Parameters
* `image_topic` 
  * raw image topic name
* `device`
  * path to camera 

## Usage
```
$ roslaunch opr_vision opr_vision.launch 
```

## Package Structure
```
opr_vision
|-CMakeLists.txt
|-launch/
  |-camera_node.launch
  |-camera_nodelet.launch
  |-opr_vision.launch
|-package.xml
|-README.md
```
