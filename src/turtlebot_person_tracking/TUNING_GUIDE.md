# Quick Tuning Guide

## Changes Made for Better Performance

### 1. Faster Gesture Detection
**Changed:** Gesture confirmation from 4/6 frames → **3/6 frames**
- **Before:** ~0.4 seconds to detect
- **After:** ~0.2 seconds to detect
- Gestures now respond faster!

### 2. Better Following (Centering)
**Changed:** Angular gain from 0.003 → **0.005**
- Robot turns more aggressively to center person
- Added 30-pixel deadband to prevent jittery movements
- Max angular speed increased to 0.6 rad/s

### 3. Obstacle Detection Disabled
**Changed:** `obstacle_detection_enabled = False`
- Robot no longer stops for obstacles
- Follows person more smoothly
- Can re-enable by setting to `True` in code

## Current Control Parameters

```python
# Distance Control
target_distance = 1.5m
Kp_linear = 0.2
max_linear_speed = 0.3 m/s

# Centering Control  
Kp_angular = 0.005
deadband = 30 pixels
max_angular_speed = 0.6 rad/s

# Gesture Detection
gesture_threshold = 100 pixels
confirmation_frames = 3 out of 6
```

## Further Tuning (if needed)

### If robot turns too much:
```python
self.Kp_angular = 0.003  # Reduce from 0.005
```

### If robot doesn't turn enough:
```python
self.Kp_angular = 0.007  # Increase from 0.005
```

### If gestures still too slow:
```python
self.gesture_confirmation_frames = 2  # Reduce from 3
```

### If gestures trigger too easily:
```python
self.gesture_confirmation_frames = 4  # Increase from 3
```

### If robot too close/far:
```python
self.target_distance = 1.8  # Increase from 1.5m
# or
self.target_distance = 1.2  # Decrease from 1.5m
```

## To Apply Changes

After editing `main.py`, restart Terminal 3:
```bash
Ctrl+C
rosrun turtlebot_person_tracking main.py
```
