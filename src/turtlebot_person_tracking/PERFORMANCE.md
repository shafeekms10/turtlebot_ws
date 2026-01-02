# Performance Optimization Guide

## Changes Made to Reduce Lag

### 1. Increased Display Rate
**Before:** 5 Hz (updates every 200ms)  
**After:** 30 Hz (updates every 33ms)  
**Result:** Much smoother camera display

### 2. Display Every Frame
**Before:** Only showed frames when detection ran (every 3rd frame)  
**After:** Shows camera feed every frame, runs detection every 2nd frame  
**Result:** No stuttering in the video display

### 3. Smaller Detection Size
**Before:** 416x416 pixels for YOLO inference  
**After:** 320x320 pixels for YOLO inference  
**Result:** Faster detection processing (~40% speed improvement)

### 4. Frame Caching
**Before:** Processed every frame from scratch  
**After:** Caches annotated frames and reuses them  
**Result:** Reduced CPU usage

---

## How to Apply These Changes

**Just restart Terminal 3:**
```bash
# In Terminal 3
Ctrl+C
rosrun turtlebot_person_tracking main.py
```

The optimizations will take effect immediately!

---

## Expected Performance

| Metric | Before | After |
|--------|--------|-------|
| Display Rate | 5 FPS | 30 FPS |
| Detection Rate | ~1.7 FPS | ~15 FPS |
| Lag | Noticeable | Minimal |
| Smoothness | Choppy | Smooth |

---

## Further Optimization Options

If you still experience lag, try these:

### Option 1: Reduce Detection Frequency
Edit `main.py` line 181:
```python
# Detect every 3rd frame instead of every 2nd
if self.frame_count % 3 == 0:
```

### Option 2: Lower Camera Resolution
In Terminal 2, use lower resolution:
```bash
roslaunch realsense2_camera rs_camera.launch \
  align_depth:=true \
  depth_width:=424 depth_height:=240 \
  color_width:=424 color_height:=240 \
  depth_fps:=15 color_fps:=15
```

### Option 3: Reduce Detection Size Further
Edit `main.py` line 184:
```python
# Even smaller for faster processing
small = cv2.resize(self.rgb_image, (256, 256), interpolation=cv2.INTER_LINEAR)
```

### Option 4: Use GPU (If Available)
The code automatically uses GPU if available. Check if CUDA is working:
```bash
python3 -c "import torch; print(f'CUDA available: {torch.cuda.is_available()}')"
```

---

## Performance Tips

✅ **Close other applications** - Free up CPU/GPU resources  
✅ **Use GPU if available** - 5-10x faster than CPU  
✅ **Lower camera FPS** - Set to 15 FPS instead of 30 FPS  
✅ **Reduce detection frequency** - Every 3rd or 4th frame  
✅ **Smaller inference size** - 256x256 or 320x320  

---

## Troubleshooting

**Still laggy after changes?**
1. Check CPU usage: `top` or `htop`
2. Check if GPU is being used (should see "cuda" in startup logs)
3. Try lower camera resolution
4. Reduce detection frequency to every 3rd or 4th frame

**Detection too slow?**
- Increase detection interval (every 3rd or 4th frame)
- Use smaller inference size (256x256)
- Lower camera resolution

**Display choppy?**
- Make sure display rate is 30 Hz
- Check that every frame is being displayed
- Close other applications using the camera
