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

### Running the System

The camera is now opened directly from the Python code. You have **two options** to run the system:

#### Option 1: Using the Standalone Script (Recommended)

```bash
cd /home/foe-usjp/turtlebot_ws/src/turtlebot_person_tracking
./run_detection.sh
```

This script will:
- Start roscore automatically if not running
- Run the person detection node
- Open the camera from Python code
- Clean up on exit

#### Option 2: Manual Execution

**Step 1: Start ROS Core**
```bash
roscore
```

**Step 2: Run Person Detection (in a new terminal)**
```bash
source /opt/ros/noetic/setup.bash
source /home/foe-usjp/turtlebot_ws/devel/setup.bash
rosrun turtlebot_person_tracking main.py
```

The camera will open automatically when the script starts.

#### Option 3: Using Launch File (Opens camera via ROS)

If you prefer to use the ROS camera driver:
```bash
source /opt/ros/noetic/setup.bash
source /home/foe-usjp/turtlebot_ws/devel/setup.bash
roslaunch turtlebot_person_tracking person_tracking.launch
```

### What You'll See

- Console logs showing:
  - Model loading status
  - Camera initialization
  - Person detection events
  - Distance measurements (e.g., "Tracking person at distance: 2.35m")

### ROS Topics

**Published Topics:**
- `/person_detection/image` - Annotated video with detection boxes
- `/cmd_vel` - Robot velocity commands (for future navigation)

**Note:** When running with Option 1 or 2, the camera is accessed directly via pyrealsense2, so ROS camera topics are not used. When using Option 3 (launch file), the following topics are also available:
- `/camera/color/image_raw` - RGB camera feed
- `/camera/aligned_depth_to_color/image_raw` - Aligned depth data

### Viewing Detection Results

To visualize the detection results:

```bash
# In a new terminal
rosrun image_view image_view image:=/person_detection/image
```

Or uncomment lines 187-188 in `scripts/main.py` to enable OpenCV window display.

## Configuration

### Custom Model Path

To use a different YOLO model, edit the launch file:

```xml
<node name="person_detection_node" pkg="turtlebot_person_tracking" type="main.py" output="screen">
  <param name="model_path" value="/path/to/your/model.pt" />
</node>
```

### Camera Settings

Camera resolution and frame rate can be adjusted in `launch/person_tracking.launch`:
- Default: 640x480 @ 15fps
- Modify `depth_width`, `depth_height`, `color_width`, `color_height`, `depth_fps`, `color_fps`

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
- Reduce camera resolution in launch file
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
