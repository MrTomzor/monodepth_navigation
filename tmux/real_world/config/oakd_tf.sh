#!/bin/bash

ros2 run tf2_ros static_transform_publisher \
  --x 0.18 \
  --y 0.0 \
  --z -0.06 \
  --roll 3.1415 \
  --pitch 0.3491 \
  --yaw 0.0 \
  --frame-id $UAV_NAME/fcu \
  --child-frame-id oak-d-base-frame
