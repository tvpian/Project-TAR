#!/bin/bash

ros2 run apriltag_ros apriltag_node --ros-args --remap camera_info:=/camera_info2  --remap image_rect:=/camera --params-file /home/rapter/project_tar/rapter_ws/px4_ws/src/apriltag_ros/cfg/tagyaml.yaml  

