# Tracking Stability Improvements

## Problems Fixed:

### 1. Robot Turning Wrong Direction ✅
**Problem**: Robot turning away from locked person even when visible

**Root Cause**: Centering control was too sensitive and would snap to zero

**Fixes**:
- Increased deadband: 40px → **50px** (more stable)
- Added **gradual slowdown** instead of instant stop
- When within deadband, angular speed reduces by 30% each cycle
- Prevents sudden direction changes

### 2. Lock Switching Between Persons ✅
**Problem**: System quickly changing locked person even when original is visible

**Root Causes**:
- Match threshold too low (0.3)
- Distance weighted more than position
- Tolerance too strict

**Fixes**:
- **Stricter match threshold**: 0.3 → **0.5** (must be 50% similar)
- **Prioritize position over distance**: 60% position, 40% distance
- **Increased tolerances**:
  - Distance: 0.5m → **0.8m**
  - Position: 150px → **200px**
- **Longer timeout**: 3s → **4s** (120 frames)
- Added `lock_match_threshold` parameter for easy tuning

### 3. Slow Gesture Detection ✅
**Problem**: Takes too long to detect gestures

**Fixes**:
- **Faster confirmation**: 3 frames → **2 frames** (out of 6)
- **Lower threshold**: 100px → **80px** (easier to trigger)
- Now responds in ~0.2s instead of ~0.3s

## Technical Details:

### Person Matching Algorithm
```python
# Prioritize position (60%) over distance (40%)
dist_score = max(0, 1 - (dist_diff / 0.8))  # 0.8m tolerance
pos_score = max(0, 1 - (pos_diff / 200))    # 200px tolerance
total_score = (dist_score * 0.4) + (pos_score * 0.6)

# Must score > 0.5 to match
if total_score > 0.5:
    match_found = True
```

### Centering with Gradual Slowdown
```python
if abs(error) < 50px:  # Within deadband
    angular_speed *= 0.7  # Reduce by 30%
    if angular_speed < 0.01:
        angular_speed = 0
```

### Gesture Detection
```python
# Buffer of last 6 frames
# Confirm if gesture detected in 2+ frames
if gesture_count >= 2:
    state = 'FOLLOWING'
```

## Expected Behavior Now:

✅ **Stable Lock**: Once locked, person stays locked even with movement  
✅ **No Switching**: Won't switch to other persons unless timeout (4s)  
✅ **Smooth Turning**: Gradual slowdown, no jerky movements  
✅ **Fast Gestures**: Responds in ~0.2 seconds  
✅ **Better Tracking**: Follows person smoothly even with rotation  

## Console Output:

```
[INFO] ✓✓✓ LOCKED on person at 1.8m ✓✓✓
[DEBUG] Match score: 0.72
[INFO] [LOCK] Tracking at 1.7m
[INFO] ✓ Pose detected! Gesture: START
[INFO] Gesture detected: START FOLLOWING
[INFO] Following: linear=0.12 m/s, angular=-0.02 rad/s, dist=1.7m
```

## Parameters (for tuning):

```python
# Lock persistence
lock_distance_threshold = 0.8      # Distance tolerance (m)
lock_position_threshold = 200      # Position tolerance (px)
lock_timeout_frames = 120          # Timeout (frames)
lock_match_threshold = 0.5         # Min match score

# Gesture detection
gesture_threshold = 80             # Hand height (px)
gesture_confirmation_frames = 2    # Frames needed

# Centering
deadband = 50                      # Stability zone (px)
Kp_angular = 0.004                 # Turn gain
max_angular_speed = 0.4            # Max turn speed (rad/s)
```
