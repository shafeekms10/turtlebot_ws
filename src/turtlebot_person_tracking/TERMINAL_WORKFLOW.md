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

### Terminal 3: Open Image Viewer (Optional)
```bash
source /opt/ros/noetic/setup.bash
rosrun image_view image_view image:=/person_detection/image
```
**What this does:** Opens a window showing the camera feed with person detection boxes and distances.

---

### Terminal 4: Run Person Detection Script
```bash
source /opt/ros/noetic/setup.bash
source /home/foe-usjp/turtlebot_ws/devel/setup.bash
rosrun turtlebot_person_tracking main.py
```
**What this does:** Runs the Python script that detects and tracks persons using the camera data.

---

## Quick Reference

| Terminal | Command | Purpose |
|----------|---------|---------|
| 1 | `roslaunch turtlebot_bringup minimal.launch` | Connect Kobuki |
| 2 | `roslaunch realsense2_camera rs_camera.launch align_depth:=true` | Connect Camera |
| 3 | `rosrun image_view image_view image:=/person_detection/image` | View Detections |
| 4 | `rosrun turtlebot_person_tracking main.py` | Run Detection Script |

---

## Expected Output

**Terminal 4 (Detection Script):**
```
[INFO] Loading YOLOv8 model from: .../yolov8n.pt
[INFO] YOLOv8 model loaded successfully!
[INFO] Person Detection Node initialized successfully!
[INFO] Waiting for camera data...
[INFO] Starting person detection loop...
[INFO] Tracking person at distance: 1.23m
[INFO] Tracking person at distance: 1.45m
```

**Terminal 3 (Image Viewer):**
- Window showing camera feed
- Green boxes around detected persons
- Distance labels on each person

---

## Stopping the System

Press `Ctrl+C` in each terminal in reverse order:
1. Terminal 4 (Detection script)
2. Terminal 3 (Image viewer)
3. Terminal 2 (Camera)
4. Terminal 1 (Kobuki)
