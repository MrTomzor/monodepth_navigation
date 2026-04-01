#!/bin/bash
### BEGIN INIT INFO
# Provides: tmux
# Required-Start:    $local_fs $network dbus
# Required-Stop:     $local_fs $network
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: start the uav
### END INIT INFO
if [ "$(id -u)" == "0" ]; then
  exec sudo -u mrs "$0" "$@"
fi

source $HOME/.bashrc

# location for storing the bag files
# * do not change unless you know what you are doing
MAIN_DIR="$HOME/bag_files"

# the project name
# * is used to define folder name in ~/$MAIN_DIR
PROJECT_NAME=sprind

# the name of the TMUX session
# * can be used for attaching as 'tmux a -t <session name>'
SESSION_NAME=mav

# following commands will be executed first in each window
# * do NOT put ; at the end
pre_input="export RMW_IMPLEMENTATION=rmw_zenoh_cpp; export USE_SIM_TIME=false"

# define commands
# 'name' 'command'
# * DO NOT PUT SPACES IN THE NAMES
# * "new line" after the command    => the command will be called after start
# * NO "new line" after the command => the command will wait for user's <enter>
input=(
  'Rosbag' 'waitForOffboard; ./record.sh
'
  'HwApi' 'ros2 launch mrs_uav_px4_api api.launch.py
'
  'Oakd' 'ros2 launch depthai_ros_driver camera.launch.py rectify_rgb:=false
'
  'Livox' 'ros2 launch livox_ros_driver2 livox.launch.py custom_config:=./config/livox.yaml json_config:=./config/dual_mid360.json
'
  'LIO_front' 'ros2 launch point_lio point_lio.launch.py preset:=mid360 topic_imu:=livox/lidar_front/imu topic_livox:=livox/lidar_front/custom custom_config:=./config/point_lio.yaml node_name:=point_lio_front
'
  'LIO_back' 'ros2 launch point_lio point_lio.launch.py preset:=mid360 topic_imu:=livox/lidar_back/imu topic_livox:=livox/lidar_back/custom custom_config:=./config/point_lio.yaml node_name:=point_lio_back
'
  'VIO' 'export DOCKER_HOST=tcp://192.168.1.181:2375 && cd ./mrs_vio/lazydocker/vio1 && ./up.sh
'
  'LZD' 'export DOCKER_HOST=tcp://192.168.1.181:2375 && lazydocker
'
  'LIO_rep_front' 'ros2 launch mrs_odometry_republisher odometry_republisher.launch.py custom_config:=./config/republisher_pointlio.yaml new_parent_frame:=odometry tf_parent:=$UAV_NAME/fcu tf_child:=$UAV_NAME/livox_front node_name:=republisher_pointlio_front topic_in:=/$UAV_NAME/point_lio_front/odometry topic_out:=~/odom
'
  'LIO_rep_back' 'ros2 launch mrs_odometry_republisher odometry_republisher.launch.py custom_config:=./config/republisher_pointlio.yaml new_parent_frame:=odometry tf_parent:=$UAV_NAME/fcu tf_child:=$UAV_NAME/livox_back node_name:=republisher_pointlio_back topic_in:=/$UAV_NAME/point_lio_back/odometry topic_out:=~/odom
'
  'VIO_rep_front' 'ros2 launch mrs_odometry_republisher odometry_republisher.launch.py custom_config:=./config/republisher_vio.yaml new_parent_frame:=odometry topic_in:=/vio1_front/open_vins/odomimu node_name:=republisher_vio_front topic_out:=~/odom
'
  'VIO_rep_back' 'ros2 launch mrs_odometry_republisher odometry_republisher.launch.py custom_config:=./config/republisher_vio.yaml new_parent_frame:=odometry topic_in:=/vio1_back/open_vins/odomimu node_name:=republisher_vio_back topic_out:=~/odom
'
  'Status' 'ros2 run mrs_uav_status status.sh
'
  'Core' 'sleep 5; ros2 launch mrs_uav_core core.launch.py platform_config:=./config/platform_config.yaml world_config:=./config/world_config.yaml custom_config:=./config/custom_config.yaml network_config:=./config/network_config.yaml
'
  'AutoStart' 'ros2 launch mrs_uav_autostart automatic_start.launch.py
'
  'LOSOS' 'sleep 10; ros2 launch mrs_losos_server losos.launch.py custom_config:=./config/losos_config.yaml lidar_3d_topic_0_in:=livox/lidar_front/points lidar_3d_topic_1_in:=livox/lidar_back/points world_frame_id:=${UAV_NAME}/stable_origin map_frame_id:=${UAV_NAME}/stable_origin semantic_pc_topic_in:=semantic_node/semantic/point_cloud
'
  'Analyzer' 'sleep 10; ros2 launch semantic_map_analyzer semantic_map_analyzer.launch.py
'
'BT_TREE' 'ros2 launch uav_bt_executor ros_node_launch.py custom_config:=./config/bt_config.yaml bt_tree_path:=./config/sprind_trees/SAR_tree.xml
'
  'BT_MONITOR' './config/tree_vis_start.sh
'
  'Semantic' 'sleep 10; ros2 launch sprind_core semantic_node.launch.py
'
  # 'Semantic' 'sleep 10; ros2 launch semantic_node_cpp semantic_node.launch.py
# '
  'Ground' 'sleep 10; ros2 launch csf_ground_filter csf_ground_filter.launch.py
# '
#   'HouseNum' 'sleep 10; ros2 launch number_recognizer house_numbers.launch.py
# '
  'PersonDet' 'sleep 10; ros2 launch yolo_detection_node yolo_detection.launch.py
'
  'PersonProj' 'ros2 launch camera_projection_node camera_projection.launch.py
'
  'PersonEst' 'ros2 launch person_state_estim state_estim.launch.py
'
  'PersonPred' 'ros2 launch person_state_predict state_predict.launch.py
'
  'PersonPlanner' 'ros2 launch path_planner planner.launch.py
'
  'Monodepth' 'ros2 launch monodepth_navigation monodepth.launch.py uav_name:=$UAV_NAME
'
  'Navigation' 'ros2 launch monodepth_navigation navigation.launch.py uav_name:=$UAV_NAME x_octogoal:=100.0 y_octogoal:=0.0 z_octogoal:=2.0 yaw_octogoal:=0.0
'
  'Undistorter' 'ros2 launch monodepth_navigation undistorter.launch.py uav_name:=$UAV_NAME use_custom:=false
'
  'Map_Plan' 'ros2 launch mrs_octomap_mapping_planning mapplan.launch.py custom_config:=./config/mapplan_config.yaml lidar_3d_0:=/midas/pointcloud_by_map uav_name:=$UAV_NAME
'

# do NOT modify the command list below
  'EstimDiag' 'waitForCore; ros2 topic echo /'"$UAV_NAME"'/estimation_manager/diagnostics --flow-style
'
  'kernel_log' 'tail -f /var/log/kern.log -n 100
'
  'zenoh' 'ros2 run rmw_zenoh_cpp rmw_zenohd
'
  'livox_tf_front' './config/livox_tf_front.sh
'
  'livox_tf_back' './config/livox_tf_back.sh
'
  'oak_tf' './config/oakd_tf.sh
'
)

# the name of the window to focus after start
init_window="Status"

# automatically attach to the new session?
# {true, false}, default true
attach=true

###########################
### DO NOT MODIFY BELOW ###
###########################

export TMUX_BIN="/usr/bin/tmux -L mrs -f /etc/ctu-mrs/tmux.conf"

# find the session
FOUND=$( $TMUX_BIN ls | grep $SESSION_NAME )

if [ $? == "0" ]; then
  echo "The session already exists"
  $TMUX_BIN -2 attach-session -t $SESSION_NAME
  exit
fi

# Absolute path to this script. /home/user/bin/foo.sh
SCRIPT=$(readlink -f $0)
# Absolute path this script is in. /home/user/bin
SCRIPTPATH=`dirname $SCRIPT`

TMUX= $TMUX_BIN new-session -s "$SESSION_NAME" -d
echo "Starting new session."

# get the iterator
ITERATOR_FILE="$MAIN_DIR/$PROJECT_NAME"/iterator.txt
if [ -e "$ITERATOR_FILE" ]
then
  ITERATOR=`cat "$ITERATOR_FILE"`
  ITERATOR=$(($ITERATOR+1))
else
  echo "iterator.txt does not exist, creating it"
  mkdir -p "$MAIN_DIR/$PROJECT_NAME"
  touch "$ITERATOR_FILE"
  ITERATOR="1"
fi
echo "$ITERATOR" > "$ITERATOR_FILE"

# create file for logging terminals' output
LOG_DIR="$MAIN_DIR/$PROJECT_NAME/"
SUFFIX=$(date +"%Y_%m_%d_%H_%M_%S")
SUBLOG_DIR="$LOG_DIR/"$ITERATOR"_"$SUFFIX""
TMUX_DIR="$SUBLOG_DIR/tmux"
mkdir -p "$SUBLOG_DIR"
mkdir -p "$TMUX_DIR"

# link the "latest" folder to the recently created one
rm "$LOG_DIR/latest" > /dev/null 2>&1
rm "$MAIN_DIR/latest" > /dev/null 2>&1
ln -sf "$SUBLOG_DIR" "$LOG_DIR/latest"
ln -sf "$SUBLOG_DIR" "$MAIN_DIR/latest"

# create arrays of names and commands
for ((i=0; i < ${#input[*]}; i++));
do
  ((i%2==0)) && names[$i/2]="${input[$i]}"
  ((i%2==1)) && cmds[$i/2]="${input[$i]}"
done

# run tmux windows
for ((i=0; i < ${#names[*]}; i++));
do
  $TMUX_BIN new-window -t $SESSION_NAME:$(($i+1)) -n "${names[$i]}"
done

sleep 3

# start loggers
for ((i=0; i < ${#names[*]}; i++));
do
  $TMUX_BIN pipe-pane -t $SESSION_NAME:$(($i+1)) -o "ts | cat >> $TMUX_DIR/$(($i+1))_${names[$i]}.log"
done

# send commands
for ((i=0; i < ${#cmds[*]}; i++));
do
  $TMUX_BIN send-keys -t $SESSION_NAME:$(($i+1)) "cd $SCRIPTPATH;
${pre_input};
${cmds[$i]}"
done

# identify the index of the init window
init_index=0
for ((i=0; i < ((${#names[*]})); i++));
do
  if [ ${names[$i]} == "$init_window" ]; then
    init_index=$(expr $i + 1)
  fi
done

$TMUX_BIN select-window -t $SESSION_NAME:$init_index

if $attach; then

  if [ -z ${TMUX} ];
  then
    $TMUX_BIN -2 attach-session -t $SESSION_NAME
  else
    tmux detach-client -E "tmux -L mrs a -t $SESSION_NAME"
  fi
else
  echo "The session was started"
  echo "You can later attach by calling:"
  echo "  tmux -L mrs a -t $SESSION_NAME"
fi
