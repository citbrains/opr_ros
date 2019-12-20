# hajime_walk_ros

## Overview
* hajime walk control package

## Usage
```
$ roslaunch hajime_walk_ros hajime_walk.launch
```
## ROS API
### Subscribed topics
* /imu/data
  * Topic to receive imu data for gyro feedback

* /hajime_walk/walk
  * Topic to receive walk command
  * Topic type: `hajime_walk/WalkCommand`
    * int32 `num_step`: number of steps to take (0 is infinite)
    * int32 `period`: walking period
    * int32 `stride_x`: stride width
    * int32 `stride_y`: stride side step
    * int32 `stride_th`: stride angle

* /hajime_walk/cancel
  * Topic to receive walk cancel command
  * Topic type: `std_msgs/Bool`
  * Execute cancel command when topic is published regardless of the actual data

* /hajime_walk/motion
  * Topic to receive motion command
  * Topic type: `hajime_walk/WalkCommand`
    * int32 `no_motion`: motion number
    * int32 `num_repeat`: number of motions to repeat (0 is infinite)

### Published topics
* `/XXXXX_controller/command`
  * Topic to publish joint angles to controller.
  * Joint angles are defined in `opr_ros_control`

### Parameters
* `motion_path`
  * file path to the robot's motion
  * optional

* eeprom_list.yaml
  * set of walking parameters

## Package Structure
```
opr_bringup
|-CMakeLists.txt
|-config/
  |-eeprom_list.yaml
|-include/
  |-acc.h
  |-calc_deg.h
  |-calc_mv.h
  |-cntr.h
  |-func.h
  |-gyro.h
  |-joy.h
  |-kine.h
  |-motion.h
  |-mvtbl.h
  |-pc_motion.h
  |-serv.h
  |-serv_init.h
  |-servo_rs.h
  |-sq_motion.h
  |-sq_ready.h
  |-sq_start.h
  |-sq_straight.h
  |-sq_walk.h
  |-var.h
|-launch/
  |-hajime_walk.launch
|-msg/
  |-MotionCommand.msg
  |-WalkCommand.msg
|-package.xml
|-README.md
|-src/
  |-acc.c
  |-calc_deg.c
  |-calc_mv.c
  |-cntr.c
  |-func.c
  |-gyro.c
  |-joy.c
  |-kine.c
  |-main.cpp
  |-motion.c
  |-mvtbl.c
  |-pc_motion.c
  |-serv.c
  |-serv_init.c
  |-sq_motion.c
  |-sq_ready.c
  |-sq_start.c
  |-sq_straight.c
  |-sq_walk.c
```
