# Complete System User Guide

## Overview

The TurtleBot person following system now includes:
- **Task A**: Person detection and locking
- **Task B**: Depth-based distance control (1.5m safety zone)
- **Task C**: Robot navigation with obstacle avoidance
- **Task D**: Hand gesture recognition for start/stop control

## How to Run

### Terminal 1: Connect Kobuki Robot
```bash
roslaunch turtlebot_bringup minimal.launch
```

### Terminal 2: Connect RealSense Camera
```bash
roslaunch realsense2_camera rs_camera.launch align_depth:=true
```

### Terminal 3: Run Person Following System
```bash
source /home/foe-usjp/turtlebot_ws/devel/setup.bash
rosrun turtlebot_person_tracking main.py
```

## System States

### IDLE State (Orange)
- Robot is NOT following
- Waiting for gesture to start
- Robot is stopped

### FOLLOWING State (Green)
- Robot IS following locked person
- Maintaining 1.5m distance
- Centering person in frame
- Avoiding obstacles

## Hand Gesture Controls

### Start Following
**Raise RIGHT hand above shoulder**
- Gesture must be held for 4+ frames (stability check)
- System switches to FOLLOWING state
- Robot begins following

### Stop Following
**Raise LEFT hand above shoulder**
- Gesture must be held for 4+ frames
- System switches to IDLE state
- Robot stops immediately

## Visual Indicators

### Person Boxes
- 🔵 **Blue box** = Locked person (target)
- 🟡 **Yellow box** = Other detected persons

### Skeleton Display
- Green lines = Skeleton connections
- Cyan dots = Body keypoints
- **Green large dot** = Right wrist (when raised for START)
- **Red large dot** = Left wrist (when raised for STOP)

### Status Display
- Top left: **FOLLOWING** (green) or **IDLE** (orange)
- Below status: **Gesture: START/STOP** (when detected)
- Bottom: Keyboard controls

## Robot Behavior

### Distance Control (Task B)
| Distance | Robot Action |
|----------|--------------|
| < 0.5m | Emergency backup |
| 0.5-1.0m | Slow backup |
| 1.0-2.0m | Maintain position |
| ~1.5m | **Target distance** |
| > 2.0m | Move forward |

### Centering Control (Task C)
- Robot turns to keep person in center of frame
- Smooth angular velocity control
- Max turn speed: 0.5 rad/s

### Obstacle Avoidance (Task C)
- Scans front 60° arc using depth camera
- Stops if obstacle < 0.8m detected
- Resumes when obstacle clears

## Keyboard Controls

| Key | Action |
|-----|--------|
| **L** | Manually lock on closest person |
| **U** | Unlock current person |
| **R** | Reset system (clear gestures, enable auto-lock) |

## Safety Features

✅ **Emergency stop** at 0.5m  
✅ **Obstacle detection** at 0.8m  
✅ **Max speed limits** (0.3 m/s linear, 0.5 rad/s angular)  
✅ **6-frame gesture buffer** (prevents false triggers)  
✅ **Auto-stop** when person lost  
✅ **Smooth acceleration/deceleration**  

## Typical Usage Workflow

1. **Start system** (3 terminals)
2. **Person enters frame** → Auto-locks (blue box)
3. **Raise right hand** → System starts FOLLOWING
4. **Walk around** → Robot follows at 1.5m, stays centered
5. **Stop suddenly** → Robot stops before collision
6. **Raise left hand** → Robot stops FOLLOWING
7. **Walk away** → Robot stays in IDLE state

## Troubleshooting

### Robot not following
- Check if state is FOLLOWING (green)
- Raise right hand to start
- Ensure person is locked (blue box)

### Gesture not detected
- Make sure hand is clearly above shoulder
- Hold gesture for ~0.5 seconds (6 frames)
- Face the camera
- Ensure good lighting

### Robot too close/far
- Target distance is 1.5m
- Adjust `target_distance` in code if needed
- Check depth camera is working

### Skeleton not showing
- Ensure person is fully visible
- Check YOLO-pose model loaded correctly
- Look for pose detection errors in console

## Parameters (Advanced)

Edit in `main.py`:

```python
self.target_distance = 1.5  # Target following distance (meters)
self.Kp_linear = 0.2  # Distance control gain
self.Kp_angular = 0.003  # Centering control gain
self.max_linear_speed = 0.3  # Max forward speed (m/s)
self.gesture_threshold = 100  # Hand raise height (pixels)
self.gesture_confirmation_frames = 4  # Frames needed to confirm gesture
```

## Console Output Examples

```
[INFO] Auto-locked on person at 1.5m
[INFO] Gesture detected: START FOLLOWING
[INFO] Obstacle detected - stopping
[WARN] Locked person lost - timeout exceeded
[INFO] Gesture detected: STOP FOLLOWING
```
