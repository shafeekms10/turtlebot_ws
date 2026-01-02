# Person Detection System - Quick Reference

## Run the System

```bash
source /opt/ros/noetic/setup.bash
source /media/psf/Home/Documents/Projects/Kobuki_Robot/kobuki_ws/devel/setup.bash
roslaunch person_detection person_detection.launch
```

## What You'll See

- OpenCV window with person detection
- Green boxes around tracked persons
- Distance measurements in meters
- Console logs with tracking info

## Complete Setup & Documentation

For detailed setup instructions, hardware initialization, troubleshooting, and system architecture, see:

**[SETUP_GUIDE.md](file:///Users/Shafeek/Documents/Projects/Kobuki_Robot/SETUP_GUIDE.md)**

This guide includes:
- First time installation
- Hardware initialization (Kobuki + RealSense D435)
- Running the system
- Testing & verification
- Troubleshooting
- System architecture
- Integration with Tasks B, C, D
