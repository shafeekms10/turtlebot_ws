#!/bin/bash

# Person Detection Standalone Runner
# This script runs the person detection without needing the camera launch file

echo "========================================="
echo "  TurtleBot Person Detection System"
echo "========================================="
echo ""

# Source ROS environment
source /opt/ros/noetic/setup.bash
source /home/foe-usjp/turtlebot_ws/devel/setup.bash

echo "Starting ROS core..."
# Check if roscore is already running
if ! pgrep -x "roscore" > /dev/null; then
    roscore &
    ROSCORE_PID=$!
    sleep 3
    echo "ROS core started (PID: $ROSCORE_PID)"
else
    echo "ROS core already running"
fi

echo ""
echo "Starting person detection node..."
echo "Camera will open automatically from Python code"
echo ""
echo "Press Ctrl+C to stop"
echo ""

# Run the detection node
rosrun turtlebot_person_tracking main.py

# Cleanup
echo ""
echo "Shutting down..."
if [ ! -z "$ROSCORE_PID" ]; then
    kill $ROSCORE_PID 2>/dev/null
    echo "ROS core stopped"
fi
