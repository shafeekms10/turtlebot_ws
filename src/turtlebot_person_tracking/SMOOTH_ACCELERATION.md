# Smooth Acceleration & Obstacle Pause

## Feature 1: Smooth Acceleration ✅

### When It Activates:
1. **START gesture** (right hand raised)
2. **Obstacle cleared** (auto-resume)

### Behavior:
- Gradual speed increase from 0 to target
- Acceleration rate: **0.04 m/s per cycle**
- Linear: Increases by 0.04 m/s each cycle
- Angular: Increases by 30% of difference each cycle
- Smooth, comfortable start

### Console Output:
```
✓ Gesture: START FOLLOWING (smooth acceleration)
Acceleration complete - at target speed
```

## Feature 2: Obstacle Pause (Not Reset!) ✅

### OLD Behavior (Reset):
```
Obstacle detected → Reset to IDLE
Person lock cleared
Need new START gesture
```

### NEW Behavior (Pause):
```
Obstacle detected → PAUSE
Person stays LOCKED
Auto-resumes when clear
```

### Detailed Flow:

**When Obstacle Appears (<50cm):**
1. Warning: `⚠ OBSTACLE <50cm - PAUSING (person still locked) ⚠`
2. Robot smoothly decelerates to stop
3. Person **remains locked** (blue box)
4. Following state stays **FOLLOWING**
5. Waits for obstacle to clear

**When Obstacle Clears:**
1. Info: `✓ Obstacle cleared - RESUMING following`
2. Smooth acceleration initiated
3. Robot resumes following
4. **No gesture needed!**

## Comparison Table:

| Event | Old Behavior | New Behavior |
|-------|--------------|--------------|
| **Obstacle appears** | Reset to IDLE | Pause (keep lock) |
| **Person lock** | Cleared | **Kept** |
| **Following state** | IDLE | **FOLLOWING** |
| **When cleared** | Need START gesture | **Auto-resume** |
| **Acceleration** | Instant | **Smooth** |

## Complete User Experience:

### Scenario 1: Normal Start
```
1. Person enters → Auto-locks (blue box)
2. Raise RIGHT hand → START gesture
3. Robot smoothly accelerates (0→target over ~0.5s)
4. Follows at 1.5m distance
```

### Scenario 2: Obstacle Encountered
```
1. Robot following person
2. Object appears <50cm
3. Robot smoothly stops (person still locked)
4. Wait for object to move...
5. Object clears
6. Robot smoothly resumes (no gesture needed!)
7. Continues following
```

### Scenario 3: Manual Stop
```
1. Robot following
2. Raise LEFT hand → STOP gesture
3. Robot smoothly decelerates
4. Stops (person still locked)
5. Raise RIGHT hand → START gesture
6. Robot smoothly accelerates
7. Resumes following
```

## Technical Details:

### Smooth Acceleration Algorithm:
```python
# Linear acceleration
if current_speed < target_speed:
    current_speed += 0.04  # m/s per cycle
    
# Angular acceleration  
angular_diff = target - current
current_angular += angular_diff * 0.3  # 30% of difference

# Complete when within 0.01 m/s of target
```

### Obstacle State Machine:
```
FOLLOWING + No Obstacle:
  → Calculate speeds
  → Apply acceleration if starting
  → Move robot

FOLLOWING + Obstacle Detected:
  → Set obstacle_paused = True
  → Smooth deceleration
  → Keep person locked
  → Wait...

FOLLOWING + Obstacle Cleared:
  → Set obstacle_paused = False
  → Set is_accelerating = True
  → Smooth resume
```

## Parameters:

```python
# Smooth acceleration
acceleration_rate = 0.04      # m/s per cycle (start)
deceleration_rate = 0.05      # m/s per cycle (stop)

# Obstacle detection
obstacle_stop_distance = 0.5  # 50cm threshold
obstacle_detection_enabled = True

# State flags
is_accelerating = False       # True during smooth start
is_decelerating = False       # True during smooth stop
obstacle_paused = False       # True when paused by obstacle
```

## Benefits:

✅ **Smoother starts** - No jerky acceleration  
✅ **Intelligent obstacle handling** - Pauses, not resets  
✅ **Auto-resume** - No gesture needed after obstacle  
✅ **Better UX** - Robot feels more natural  
✅ **Keeps context** - Person stays locked through pause  
