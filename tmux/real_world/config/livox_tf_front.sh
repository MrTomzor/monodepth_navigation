#!/bin/bash

ros2 run tf2_ros static_transform_publisher \
  --x 0.06 \
  --y -0.05 \
  --z -0.16 \
  --roll -2.6179 \
  --pitch 0.0 \
  --yaw -1.57149 \
  --frame-id $UAV_NAME/fcu \
  --child-frame-id $UAV_NAME/livox_front
