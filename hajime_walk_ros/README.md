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
|-launch/
  |-hajime_walk.launch
|-msg/
  |-MotionCommand.msg
  |-WalkCommand.msg
|-package.xml
|-README.md
|-src/
  |-acc.c
  |-acc.h
  |-b3m.c
  |-b3m.h
  |-calc_deg.c
  |-calc_deg.h
  |-calc_mv.c
  |-calc_mv.h
  |-cntr.c
  |-cntr.h
  |-func.c
  |-func.h
  |-gyro.c
  |-gyro.h
  |-joy.c
  |-joy.h
  |-kine.c
  |-kine.h
  |-KSerialPort.h
  |-main.cpp
  |-motion.c
  |-motion.h
  |-mvtbl.c
  |-mvtbl.h
  |-pc_motion.c
  |-pc_motion.h
  |-serv.c
  |-serv.h
  |-serv_init.c
  |-serv_init.h
  |-sq_motion.c
  |-sq_motion.h
  |-sq_ready.c
  |-sq_ready.h
  |-sq_start.c
  |-sq_start.h
  |-sq_straight.c
  |-sq_straight.h
  |-sq_walk.c
  |-sq_walk.h
  |-var,h
  |-var_init.h
```


