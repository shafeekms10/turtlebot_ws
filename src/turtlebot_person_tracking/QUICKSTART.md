# Quick Start Guide - Person Detection

## How to Run (Camera Opens from Python Code)

### Method 1: Easiest Way ✨

```bash
cd /home/foe-usjp/turtlebot_ws/src/turtlebot_person_tracking
./run_detection.sh
```

**That's it!** The script handles everything:
- Starts ROS core if needed
- Opens the camera automatically
- Runs person detection
- Shows detection logs in terminal

Press `Ctrl+C` to stop.

---

### Method 2: Manual Steps

**Terminal 1:**
```bash
roscore
```

**Terminal 2:**
```bash
source /opt/ros/noetic/setup.bash
source /home/foe-usjp/turtlebot_ws/devel/setup.bash
rosrun turtlebot_person_tracking main.py
```

The camera opens automatically when `main.py` starts.

---

## What You'll See

Console output:
```
[INFO] Loading YOLOv8 model from: /home/foe-usjp/.../yolov8n.pt
[INFO] YOLOv8 model loaded successfully!
[INFO] Initializing RealSense camera...
[INFO] RealSense camera initialized successfully!
[INFO] Person Detection Node initialized successfully!
[INFO] Starting person detection...
[INFO] Starting person detection loop...
[INFO] Tracking person at distance: 1.23m
[INFO] Tracking person at distance: 1.45m
...
```

---

## View Detection Video (Optional)

To see the video with bounding boxes:

```bash
# In a new terminal
rosrun image_view image_view image:=/person_detection/image
```

Or uncomment lines 211-212 in `main.py` to show OpenCV window directly.

---

## Key Changes from Before

**Before:** Camera opened via ROS launch file
```bash
roslaunch turtlebot_person_tracking person_tracking.launch
```

**Now:** Camera opens from Python code directly
```bash
./run_detection.sh
# OR
rosrun turtlebot_person_tracking main.py
```

**Advantage:** Simpler workflow, no need for launch file to start camera!

---

## Troubleshooting

**Camera not found:**
```bash
# Check if camera is connected
rs-enumerate-devices
```

**Permission denied on run_detection.sh:**
```bash
chmod +x /home/foe-usjp/turtlebot_ws/src/turtlebot_person_tracking/run_detection.sh
```

**Module not found errors:**
```bash
pip3 install -r /home/foe-usjp/turtlebot_ws/src/turtlebot_person_tracking/requirements.txt
```
