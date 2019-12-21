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
|-meshes/
  |-version1/
    |-STL/
      |-ARMS/
        |-body_to_arm.stl
        |-hand.stl
        |-left_arm.stl
        |-right_arm.stl
      |-BODY/
        |-body.stl
      |-HEAD/
        |-head_pitch.stl
        |-head_yaw.stl
      |-LEGS/
        |-ankle_pitch.stl
        |-back_link.stl
        |-body_to_waist.stl
        |-front_link.stl
        |-left_ankle.stl
        |-left_foot.stl
        |-left_knee.stl
        |-right_ankle.stl
        |-right_foot.stl
        |-right_knee.stl
        |-waist.stl
  |-version3/
    |-DAE/
      |-ankle_pitch_left.dae
      |-ankle_pitch_right.dae
      |-ankle_roll_left.dae
      |-ankle_roll_right.dae
      |-arm.dae
      |-body.dae
      |-hand.dae
      |-head.dae
      |-independent_pitch.dae
      |-knee_pitch_left.dae
      |-knee_pitch_right.dae
      |-shin_pitch.dae
      |-shin_pitch_mimic.dae
      |-shoulder.dae
      |-slider_crank.dae
      |-waist_pitch.dae
      |-waist_pitch_mimic.dae
      |-waist_roll_left.dae
      |-waist_roll_right.dae
      |-waist_yaw.dae
    |-STL/
      |-ankle_pitch_left.stl
      |-ankle_pitch_right.stl
      |-ankle_roll_left.stl
      |-ankle_roll_right.stl
      |-arm.stl
      |-body.stl
      |-hand.stl
      |-head.stl
      |-independent_pitch.stl
      |-knee_pitch_left.stl
      |-knee_pitch_right.stl
      |-shin_pitch.stl
      |-shin_pitch_mimic.stl
      |-shoulder.stl
      |-slider_crank.stl
      |-waist_pitch.stl
      |-waist_pitch_mimic.stl
      |-waist_roll_left.stl
      |-waist_roll_right.stl
      |-waist_yaw.stl
|-package.xml
|-README.md
|-robots/
  |-gankenkun_v1.xacro
  |-gankenkun_v3.xacro
|-urdf/
  |-gazebo.xacro
  |-materials.xacro
  |-version1/
    |-arms/
      |-arm.gazebo.xacro
      |-arm.transmission.xacro
      |-left_arm.urdf.xacro
      |-right_arm.urdf.xacro
    |-body/
      |-body.gazebo.xacro
      |-body.urdf.xacro
    |-head/
      |-head.gazebo.xacro
      |-head.transmission.xacro
      |-head.urdf.xacro
    |-legs/
      |-left_leg.urdf.xacro
      |-leg.gazebo.urdf
      |-leg.transmission.xacro
      |-right_leg.urdf.xacro
  |-version3/
    |-arms/
      |-arm.gazebo.xacro
      |-arm.transmission.xacro
      |-left_arm.urdf.xacro
      |-right_arm.urdf.xacro
    |-body/
      |-body.gazebo.xacro
      |-body.urdf.xacro
    |-head/
      |-head.gazebo.xacro
      |-head.transmission.xacro
      |-head.urdf.xacro
    |-legs/
      |-left_leg.urdf.xacro
      |-leg.gazebo.urdf
      |-leg.transmission.xacro
      |-right_leg.urdf.xacro
    |-sensors/
      |-camera/
        |-camera.gazebo.xacro
        |-camera.urdf.xacro
      |-imu/
        |-imu.gazebo.xacro
        |-imu.urdf.xacro
```
