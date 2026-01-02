# Development Workflow - Making Code Changes

## If You Change the Python Code

### Quick Answer
**Only restart Terminal 3** (the Python script). Keep Terminal 1 (robot) and Terminal 2 (camera) running.

---

## Step-by-Step

### 1. Make Your Code Changes
Edit `scripts/main.py` in your editor and save the file.

### 2. Stop the Python Script
In **Terminal 3**, press `Ctrl+C`

This will:
- Stop the detection script
- Close the OpenCV window
- Keep the robot and camera running

### 3. Restart the Python Script
In **Terminal 3**, run the same command again:
```bash
rosrun turtlebot_person_tracking main.py
```

**That's it!** Your changes will be applied.

---

## What to Restart When

| What Changed | Restart Terminal 1 (Robot) | Restart Terminal 2 (Camera) | Restart Terminal 3 (Python) |
|--------------|----------------------------|-----------------------------|-----------------------------|
| Python code (`main.py`) | ❌ No | ❌ No | ✅ **Yes** |
| Model file (`yolov8n.pt`) | ❌ No | ❌ No | ✅ **Yes** |
| Camera settings | ❌ No | ✅ **Yes** | ✅ **Yes** (after camera) |
| Robot configuration | ✅ **Yes** | ❌ No | ✅ **Yes** (after robot) |

---

## Example Development Session

```bash
# Initial setup (do once)
Terminal 1: roslaunch turtlebot_bringup minimal.launch
Terminal 2: roslaunch realsense2_camera rs_camera.launch align_depth:=true
Terminal 3: rosrun turtlebot_person_tracking main.py

# Make code changes in editor...

# Apply changes (Terminal 3 only)
Terminal 3: Ctrl+C
Terminal 3: rosrun turtlebot_person_tracking main.py

# Make more changes...

# Apply changes again (Terminal 3 only)
Terminal 3: Ctrl+C
Terminal 3: rosrun turtlebot_person_tracking main.py
```

---

## Pro Tips

### 1. Keep Robot and Camera Running
Leave Terminal 1 and 2 running while you develop. Only restart Terminal 3 when you change code.

### 2. Quick Restart Shortcut
In Terminal 3, after pressing `Ctrl+C`, press the **Up Arrow** key to get the last command, then press **Enter**.

### 3. No Need to Rebuild
Python scripts don't need `catkin_make` to apply changes. Just restart the script.

### 4. When to Rebuild with catkin_make
Only rebuild if you:
- Add new Python files
- Change `CMakeLists.txt`
- Change `package.xml`
- Add new dependencies

For code changes in existing files, **no rebuild needed**.

---

## Common Scenarios

### Scenario 1: Testing Detection Parameters
```python
# Change confidence threshold in main.py
self.confidence_threshold = 0.7  # Changed from 0.5
```
**Action:** Restart Terminal 3 only

### Scenario 2: Adjusting Frame Processing
```python
# Process every frame instead of every 3rd
if self.frame_count % 1 != 0:  # Changed from % 3
```
**Action:** Restart Terminal 3 only

### Scenario 3: Changing Camera Resolution
```bash
# Terminal 2: Change launch command
roslaunch realsense2_camera rs_camera.launch \
  align_depth:=true \
  depth_width:=1280 depth_height:=720
```
**Action:** 
1. Stop Terminal 3 (Ctrl+C)
2. Stop Terminal 2 (Ctrl+C)
3. Restart Terminal 2 with new settings
4. Restart Terminal 3

---

## Summary

✅ **Python code changes** → Restart Terminal 3 only  
✅ **Fast iteration** → Keep robot and camera running  
✅ **No rebuild needed** → Python changes apply immediately  
✅ **Use Up Arrow** → Quick command recall in terminal
