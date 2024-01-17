#!/bin/bash

# Start roscore
xterm -e "roscore" &

# Wait for roscore to initialize
sleep 2

# Run rosbag play in a new xterm window
xterm -e "rosbag play recordings/1.0.1.bag" &

# Run slam_gmapping in another new xterm window
xterm -e "rosrun gmapping slam_gmapping scan:=/naoqi_driver/laser" &

# Get the PID (Process ID) of the last background process (slam_gmapping)
gmapping_pid=$!

# Wait for the user to manually close slam_gmapping
echo "Press Enter to save the map and exit slam_gmapping"
read -p ""

# Save the map using map_saver
rosrun map_server map_saver -f results/gmapping/1.0.1/base

# Terminate the slam_gmapping process
kill -INT $gmapping_pid
