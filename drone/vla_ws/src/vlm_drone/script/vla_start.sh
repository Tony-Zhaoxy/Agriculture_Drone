#!/usr/bin/env bash
set -e

SESSION="flightsession"
WS=~/AGRICULTURE_DRONE/drone/vla_ws

# PX4 XRCE Agent
XRCE_MODE="udp4"
XRCE_PORT=8888

# OpenVINS (EDIT THESE)
OPENVINS_LAUNCH="ov_msckf.launch.py"
OPENVINS_PKG="ov_msckf"
# example config arg (depends on your launch)
OPENVINS_ARGS="config:=rs_D455"

BAG_DIR=~/bags/vla_$(date +%Y%m%d_%H%M%S)
SRC_CMD="source /opt/ros/humble/setup.bash && source ${WS}/install/setup.bash"

tmux has-session -t "$SESSION" 2>/dev/null && tmux kill-session -t "$SESSION"
tmux new-session -d -s "$SESSION" -n "PX4"

# Window 0: PX4 agent + Camera
tmux send-keys -t ${SESSION}:0.0 "${SRC_CMD} && MicroXRCEAgent ${XRCE_MODE} -p ${XRCE_PORT}" C-m
tmux split-window -v -t ${SESSION}:0
tmux send-keys -t ${SESSION}:0.1 "${SRC_CMD} && ros2 launch vlm_drone camera.launch.py" C-m

# Window 1: OpenVINS
tmux new-window -t "$SESSION" -n "OpenVINS"
tmux send-keys -t ${SESSION}:1.0 "${SRC_CMD} && echo '[OpenVINS] launching...'" C-m
tmux send-keys -t ${SESSION}:1.0 "${SRC_CMD} && ros2 launch ${OPENVINS_PKG} ${OPENVINS_LAUNCH} ${OPENVINS_ARGS}" C-m

# Window 2: VLA (server + controller) + bag
tmux new-window -t "$SESSION" -n "VLA"
tmux send-keys -t ${SESSION}:2.0 "${SRC_CMD} && ros2 run vla_server server" C-m
tmux split-window -v -t ${SESSION}:2
tmux send-keys -t ${SESSION}:2.1 "${SRC_CMD} && ros2 run vlm_drone drone_node" C-m
tmux split-window -h -t ${SESSION}:2.1
tmux send-keys -t ${SESSION}:2.2 "${SRC_CMD} && mkdir -p $(dirname ${BAG_DIR}) && echo '[BAG] ${BAG_DIR}'" C-m

# rosbag2 topics: PX4 + images + OpenVINS (adjust OpenVINS topics after you confirm them)
tmux send-keys -t ${SESSION}:2.2 "${SRC_CMD} && ros2 bag record -o ${BAG_DIR} \
  /t265/fisheye1/image_raw \
  /vla_image \
  /tf /tf_static \
  /fmu/in/offboard_control_mode \
  /fmu/in/trajectory_setpoint \
  /fmu/in/vehicle_command \
  /fmu/out/vehicle_odometry \
  /fmu/out/vehicle_status \
  /ov_msckf/odom \
  /ov_msckf/pose" C-m

# Window 3: Debug
tmux new-window -t "$SESSION" -n "Debug"
tmux send-keys -t ${SESSION}:3.0 "${SRC_CMD} && ros2 topic hz /fmu/in/trajectory_setpoint" C-m
tmux split-window -v -t ${SESSION}:3
tmux send-keys -t ${SESSION}:3.1 "${SRC_CMD} && ros2 topic list | grep -E 'ov_|fmu|t265|image'" C-m

# QoL
tmux set-option -g mouse on
tmux set-option -g history-limit 10000

tmux attach-session -t "$SESSION"
