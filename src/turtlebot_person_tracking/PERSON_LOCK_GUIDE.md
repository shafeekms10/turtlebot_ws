# Person Lock and Tracking - User Guide

## Overview

The system now implements **person lock and tracking** where the robot:
- **Auto-locks** on the first detected person
- **Tracks only that person** for following
- **Shows locked person differently** (blue box) from others (yellow boxes)
- **Maintains tracking** through partial occlusions

## Visual Indicators

### Locked Person (Target to Follow)
- **Blue bounding box** (thick, 4px)
- Label: `LOCKED: 0.95 | 1.5m`
- Status: "LOCKED TARGET" at top of screen

### Other Detected Persons
- **Yellow bounding box** (thin, 2px)
- Label: `Person: 0.92 | 2.3m`
- Not tracked for following

### No Lock Status
- Status: "NO LOCK - Searching..." at top of screen

## Keyboard Controls

Press these keys while the OpenCV window is active:

| Key | Action | Description |
|-----|--------|-------------|
| **L** | Lock | Lock on closest detected person |
| **U** | Unlock | Unlock current target |
| **R** | Reset | Reset and enable auto-lock |

Controls are displayed at the bottom of the video window.

## How It Works

### Auto-Lock (Default)
1. System starts with auto-lock enabled
2. When first person detected → automatically locks
3. Tracks that person continuously
4. Ignores other people

### Manual Lock
1. Press `U` to unlock current person
2. Press `L` to lock on closest person
3. System tracks the newly locked person

### Tracking Continuity
- Uses distance and position matching
- Handles partial occlusions
- Re-acquires person if temporarily lost
- Timeout: 3 seconds (90 frames at 30fps)

## Console Output

```
[INFO] Auto-lock enabled - will lock on first detected person
[INFO] Auto-locked on person at 1.5m
[INFO] Tracking locked person at 1.5m
[WARN] Locked person not found, searching...
[WARN] Locked person lost - timeout exceeded
[INFO] Manually locked on person at 2.1m
[INFO] Unlocked person - ready to lock new target
[INFO] Reset - auto-lock enabled
```

## Usage Scenarios

### Scenario 1: Follow Specific Person
1. Start system (auto-lock enabled)
2. Target person enters frame → auto-locks (blue box)
3. Other people appear → shown in yellow
4. Robot follows only the blue-boxed person

### Scenario 2: Switch Target
1. Current person locked (blue box)
2. Press `U` to unlock
3. Move closer to new target person
4. Press `L` to lock on new person

### Scenario 3: Reset After Loss
1. Locked person leaves frame
2. After 3 seconds → lock lost
3. Press `R` to reset
4. Next person detected → auto-locks

## Occlusion Handling

The system handles partial occlusions:
- Maintains lock when person partially hidden
- Uses last known position for matching
- Re-acquires when person reappears
- Timeout after 3 seconds of no detection

## Tips

✅ **For best tracking**: Keep locked person in center of frame  
✅ **To switch targets**: Use `U` then `L` keys  
✅ **If tracking lost**: Press `R` to reset and auto-lock  
✅ **Multiple people**: Only locked person (blue) is tracked  
✅ **Distance shown**: For all detected persons

## Technical Details

### Matching Algorithm
- **Distance similarity**: ±0.5m threshold
- **Position similarity**: 150 pixels threshold
- **Combined score**: 60% distance + 40% position
- **Minimum match score**: 0.3

### Lock Timeout
- **Duration**: 3 seconds (90 frames)
- **Behavior**: Auto-unlock if person not found
- **Recovery**: Can re-lock when person reappears
