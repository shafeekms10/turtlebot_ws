# Terminal Workflow Guide

## Step-by-Step: Connect Everything from Terminal

### Terminal 1: Connect Kobuki Robot
```bash
roslaunch turtlebot_bringup minimal.launch
```
**What this does:** Starts the TurtleBot Kobuki base and makes it ready to receive commands.

---

### Terminal 2: Connect Camera
```bash
source /opt/ros/noetic/setup.bash
source /home/foe-usjp/turtlebot_ws/devel/setup.bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```
**What this does:** Opens the RealSense D435 camera and publishes RGB and depth streams to ROS topics.

---

### Terminal 3: Run Person Detection Script
```bash
source /opt/ros/noetic/setup.bash
source /home/foe-usjp/turtlebot_ws/devel/setup.bash
rosrun turtlebot_person_tracking main.py
```
**What this does:** 
- Runs the Python script that detects and tracks persons using the camera data
- **Opens an OpenCV window** showing the camera view with detection boxes and distances

---

## Quick Reference

| Terminal | Command | Purpose |
|----------|---------|---------|
| 1 | `roslaunch turtlebot_bringup minimal.launch` | Connect Kobuki |
| 2 | `roslaunch realsense2_camera rs_camera.launch align_depth:=true` | Connect Camera |
| 3 | `rosrun turtlebot_person_tracking main.py` | Run Detection + Show Window |

---

## Expected Output

**Terminal 3 (Detection Script):**
```
[INFO] Loading YOLOv8 model from: .../yolov8n.pt
[INFO] YOLOv8 model loaded successfully!
[INFO] Person Detection Node initialized successfully!
[INFO] Waiting for camera data...
[INFO] Starting person detection loop...
[INFO] Tracking person at distance: 1.23m
[INFO] Tracking person at distance: 1.45m
```

**OpenCV Window (Opens automatically from Python):**
- Window titled "Person Detection & Tracking"
- Live camera feed
- Green boxes around detected persons
- Distance labels on each person
- "TRACKING TARGET" label on closest person

---

## Stopping the System

Press `Ctrl+C` in Terminal 3 first (this will close the OpenCV window), then:
1. Terminal 2 (Camera)
2. Terminal 1 (Kobuki)
