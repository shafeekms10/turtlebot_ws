# TurtleBot Person Tracking

Person detection and tracking system using YOLOv8 and RealSense D435 camera for TurtleBot.

## Features

- Real-time person detection using YOLOv8
- Distance measurement using RealSense depth camera
- Automatic tracking of closest person
- ROS integration for robot control
- Visual feedback with bounding boxes and distance labels

## Prerequisites

- ROS Noetic
- Python 3
- RealSense D435 camera
- TurtleBot with Kobuki base

## Installation

### 1. Install Python Dependencies

```bash
cd /home/foe-usjp/turtlebot_ws/src/turtlebot_person_tracking
pip3 install -r requirements.txt
```

### 2. Build the Package

```bash
cd /home/foe-usjp/turtlebot_ws
catkin_make
source devel/setup.bash
```

## Usage

### Running the System - Terminal Workflow

The system requires **3 terminals**. Run commands in this order:

#### Terminal 1: Connect Kobuki Robot
```bash
roslaunch turtlebot_bringup minimal.launch
```

#### Terminal 2: Connect RealSense Camera
```bash
source /opt/ros/noetic/setup.bash
source /home/foe-usjp/turtlebot_ws/devel/setup.bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

#### Terminal 3: Run Person Detection
```bash
source /opt/ros/noetic/setup.bash
source /home/foe-usjp/turtlebot_ws/devel/setup.bash
rosrun turtlebot_person_tracking main.py
```

**Note:** An OpenCV window will automatically open showing the camera view with detection boxes and tracking details.

### Making Code Changes During Development

If you change the Python code (`main.py`), you **only need to restart Terminal 3**:
1. Press `Ctrl+C` in Terminal 3 to stop the script
2. Run `rosrun turtlebot_person_tracking main.py` again

**Keep Terminal 1 (robot) and Terminal 2 (camera) running** - no need to restart them!

See [DEVELOPMENT.md](file:///home/foe-usjp/turtlebot_ws/src/turtlebot_person_tracking/DEVELOPMENT.md) for detailed development workflow.

### What You'll See

- Console logs showing:
  - Model loading status
  - Camera initialization
  - Person detection events
  - Distance measurements (e.g., "Tracking person at distance: 2.35m")

### ROS Topics

**Subscribed Topics:**
- `/camera/color/image_raw` - RGB camera feed (from Terminal 2)
- `/camera/aligned_depth_to_color/image_raw` - Aligned depth data (from Terminal 2)

**Published Topics:**
- `/person_detection/image` - Annotated video with detection boxes (viewed in Terminal 3)

### Viewing Detection Results

An **OpenCV window** automatically opens when you run the detection script (Terminal 3), showing:
- Window title: "Person Detection & Tracking"
- Live camera feed with annotations
- Green bounding boxes around detected persons
- Distance measurements in meters
- "TRACKING TARGET" label on the closest person

The window will close automatically when you stop the script with `Ctrl+C`.

## Configuration

### Camera Settings

Camera resolution and frame rate can be adjusted when launching the camera (Terminal 2):
```bash
roslaunch realsense2_camera rs_camera.launch \
  align_depth:=true \
  depth_width:=640 depth_height:=480 \
  color_width:=640 color_height:=480 \
  depth_fps:=15 color_fps:=15
```

## Troubleshooting

### Camera Not Detected
```bash
# Check if RealSense camera is connected
rs-enumerate-devices

# Test camera
realsense-viewer
```

### No Detections
- Ensure proper lighting
- Check camera field of view
- Verify model file exists in `models/yolov8n.pt`
- Check confidence threshold (default: 0.5)

### Performance Issues
- Reduce camera resolution in Terminal 2 camera launch command
- Increase frame skip in `main.py` (currently processes every 3rd frame)
- Use GPU if available (automatically detected)

## System Architecture

```
┌─────────────────┐
│  RealSense D435 │
│     Camera      │
└────────┬────────┘
         │
         ├─ RGB Stream (/camera/color/image_raw)
         │
         └─ Depth Stream (/camera/aligned_depth_to_color/image_raw)
         │
         ▼
┌─────────────────────────┐
│ Person Detection Node   │
│  - YOLOv8 Detection     │
│  - Distance Calculation │
│  - Tracking Logic       │
└──────────┬──────────────┘
           │
           ├─ Detection Image (/person_detection/image)
           │
           └─ Velocity Commands (/cmd_vel)
```

## Performance

- Detection Rate: ~5 Hz
- Model: YOLOv8 Nano (lightweight, fast)
- Input Size: 416x416 (resized for speed)
- Frame Processing: Every 3rd frame

## License

MIT
