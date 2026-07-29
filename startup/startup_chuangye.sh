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
tmux send-keys -t mysession:0.0 'source ./devel/setup.bash' C-m
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

# Top-0.0 (roscore) is already configured


# Top-0.1 (QGC serial port)
tmux send-keys -t mysession:0.1 'source ./devel/setup.bash' C-m
# tmux send-keys -t mysession:0.1 'rosrun waypoint_pub serial_bridge.py' C-m

# Top-0.2 (px4)
tmux send-keys -t mysession:0.2 'source ./devel/setup.bash' C-m
tmux send-keys -t mysession:0.2 'roslaunch waypoint_pub px4_vicon.launch' C-m
sleep 0.5
# rostopic echo /uav0/mavros/state

# Top-0.3 (livox_driver)
tmux send-keys -t mysession:0.3 'source ./devel/setup.bash' C-m
tmux send-keys -t mysession:0.3 'roslaunch livox_ros_driver2 msg_MID360.launch' C-m
sleep 0.5

# 2nd-row-0.4 (lio)
tmux send-keys -t mysession:0.4 'source ./devel/setup.bash' C-m
tmux send-keys -t mysession:0.4 'roslaunch sfast_lio mapping_mid360.launch' C-m

# ---------------------------
# Configure the bottom row
# ---------------------------

# 2nd-row-0.5 (odom2pose)
tmux send-keys -t mysession:0.5 'source ./devel/setup.bash' C-m
tmux send-keys -t mysession:0.5 'rosrun waypoint_pub odom2pose.py' C-m
sleep 0.5


# bottom 0.6-0.8, blank, wait realsense to install

# 2nd-row-0.6 multi-ROS realsense camera
tmux send-keys -t mysession:0.6 'source ./devel/setup.bash' C-m
tmux send-keys -t mysession:0.6 'roslaunch pointcloud_to_laserscan cloud_to_scan_obstacle.launch' C-m

tmux send-keys -t mysession:0.7 'source ./devel/setup.bash' C-m
# tmux send-keys -t mysession:0.7 'roslaunch realsense2_camera rs_camera.launch' C-m

tmux send-keys -t mysession:0.8 'source ./devel/setup.bash' C-m
# tmux send-keys -t mysession:0.8 'rosrun waypoint_pub odom2pose.py' C-m

# ---------------------------
# Final layout adjustments
# ---------------------------

# Set equal pane sizes
tmux select-layout -t mysession:0 tiled

# Focus on the blank 
tmux select-pane -t mysession:0.6

# Attach to session
tmux attach-session -t mysession

#test if git recognize links