#!/bin/bash

ros2 run tf2_ros static_transform_publisher \
  --x 0.0 \
  --y 0.0 \
  --z -0.15 \
  --roll 0.0 \
  --pitch 0.7854 \
  --yaw 0.0 \
  --frame-id $UAV_NAME/fcu \
  --child-frame-id uav1/mrs_vio_imu
