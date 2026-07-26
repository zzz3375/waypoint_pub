#!/bin/bash

# Create new tmux session
tmux new-session -d -s mysession -n ros_session

# Kill old ROS processes
tmux send-keys -t mysession 'killall -9 rosmaster roscore rosout' C-m
sleep 1

# ---------------------------
# Create the grid layout first
# ---------------------------

# Create initial pane (will become top-left)
# Create initial pane
tmux send-keys -t mysession:0.0 'source ../../../devel/setup.bash' C-m
tmux send-keys -t mysession:0.0 'roscore' C-m
sleep 2

# First split: vertical (creates top and bottom rows)
tmux split-window -v -t mysession:0.0

# Split top row into 4 columns
tmux split-window -h -t mysession:0.0
tmux split-window -h -t mysession:0.1
tmux split-window -h -t mysession:0.2

# Split bottom row into 4 columns
tmux split-window -h -t mysession:0.4
tmux split-window -h -t mysession:0.5
tmux split-window -h -t mysession:0.6

# Now you have:
# 0.0 0.1 0.2 0.3 (top row)
# 0.4 0.5 0.6 0.7 (bottom row)
# ---------------------------
# Configure the top row
# ---------------------------

# Top-left (roscore) is already configured

# Top-middle (px4)
tmux send-keys -t mysession:0.1 'source ../../../devel/setup.bash' C-m
tmux send-keys -t mysession:0.1 'roslaunch waypoint_pub px4_vicon.launch' C-m
sleep 0.5
# rostopic echo /uav0/mavros/state

# Top-right (livox_driver)
tmux send-keys -t mysession:0.2 'source ../../../devel/setup.bash' C-m
tmux send-keys -t mysession:0.2 'roslaunch livox_ros_driver2 msg_MID360.launch' C-m
sleep 0.5

# 2nd-row-left (lio)
tmux send-keys -t mysession:0.3 'source ../../../devel/setup.bash' C-m
tmux send-keys -t mysession:0.3 'roslaunch sfast_lio mapping_mid360.launch' C-m

# 2nd-row-middle (lio)
tmux send-keys -t mysession:0.4 'source ../../../devel/setup.bash' C-m
# tmux send-keys -t mysession:0.4 'rosrun waypoint_pub odom2pose.py' C-m

# ---------------------------
# Configure the bottom row
# ---------------------------

# multi-ROS realsense camera
tmux send-keys -t mysession:0.5 'source ../../../devel/setup.bash' C-m
# tmux send-keys -t mysession:0.5 'export ROS_MASTER_URI=http://192.168.50.165:11311' C-m
# tmux send-keys -t mysession:0.5 'export ROS_IP=192.168.50.67' C-m
tmux send-keys -t mysession:0.5 'roslaunch realsense2_camera rs_camera.launch' C-m

tmux send-keys -t mysession:0.6 'source ../../../devel/setup.bash' C-m
# tmux send-keys -t mysession:0.6 'roslaunch realsense2_camera rs_camera.launch' C-m

tmux send-keys -t mysession:0.7 'source ../../../devel/setup.bash' C-m
tmux send-keys -t mysession:0.7 'rosrun waypoint_pub odom2pose.py' C-m

# ---------------------------
# Final layout adjustments
# ---------------------------

# Set equal pane sizes
tmux select-layout -t mysession:0 tiled

# Focus on the realsense pane (0.6)
tmux select-pane -t mysession:0.6

# Attach to session
tmux attach-session -t mysession
