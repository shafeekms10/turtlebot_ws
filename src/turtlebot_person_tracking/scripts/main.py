#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import torch
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO
import os
from collections import deque, Counter

class PersonDetectionNode:
    def __init__(self):
        rospy.init_node('person_detection_node', anonymous=True)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.bridge = CvBridge()
        
        # Get model path from ROS parameter or use default
        model_path = rospy.get_param('~model_path', 
            os.path.join(os.path.dirname(__file__), '..', 'models', 'yolov8n.pt'))
        
        rospy.loginfo(f"Loading YOLOv8 model from: {model_path}")
        self.model = YOLO(model_path)
        rospy.loginfo("YOLOv8 model loaded successfully!")
        
        self.rgb_image = None
        self.depth_image = None
        self.latest_detection = None
        self.frame_count = 0
        
        # Subscribe to camera topics
        self.rgb_sub = rospy.Subscriber(
            '/camera/color/image_raw', 
            Image, 
            self.rgb_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        self.depth_sub = rospy.Subscriber(
            '/camera/aligned_depth_to_color/image_raw', 
            Image, 
            self.depth_callback,
            queue_size=1,
            buff_size=2**24
        )
        
        # Publisher for robot control
        # TurtleBot uses /mobile_base/commands/velocity
        self.cmd_vel_pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=10)
        
        # Publisher for detection visualization
        self.detection_image_pub = rospy.Publisher(
            '/person_detection/image', 
            Image, 
            queue_size=10
        )
        
        self.confidence_threshold = 0.5
        self.person_class_id = 0
        
        # Person lock and tracking state
        self.locked_person = None  # Stores locked person info
        self.lock_enabled = False  # Lock state flag
        self.auto_lock = True  # Auto-lock first person
        self.lock_distance_threshold = 0.8  # meters (increased tolerance)
        self.lock_position_threshold = 200  # pixels (increased tolerance)
        self.lock_timeout_frames = 120  # 4 seconds at 30fps (more forgiving)
        self.lock_match_threshold = 0.5  # Minimum score to match locked person
        
        # Load YOLO-Pose model for gesture recognition
        pose_model_path = os.path.join(os.path.dirname(__file__), '..', 'models', 'yolov8n-pose.pt')
        rospy.loginfo(f"Loading YOLOv8-Pose model from: {pose_model_path}")
        self.pose_model = YOLO(pose_model_path)
        rospy.loginfo("YOLOv8-Pose model loaded successfully!")
        
        # Navigation and gesture control state
        self.following_state = 'IDLE'  # 'IDLE' or 'FOLLOWING'
        self.gesture_buffer = deque(maxlen=6)  # 6-frame buffer for gesture stability
        self.current_linear_speed = 0.0
        self.current_angular_speed = 0.0
        
        # Distance control parameters (Task B)
        self.target_distance = 1.5  # meters
        self.Kp_linear = 0.2  # Proportional gain for distance control
        self.Kd_linear = 0.05  # Derivative gain
        self.max_linear_speed = 0.3  # m/s
        self.min_linear_speed = -0.1  # m/s (backup)
        self.emergency_stop_distance = 0.5  # meters
        self.last_distance_error = 0.0
        
        # Centering control parameters (Task C)
        self.Kp_angular = 0.004  # rad/s per pixel (smooth turning)
        self.max_angular_speed = 0.4  # rad/s (reduced for smoother turns)
        self.angular_acceleration_limit = 0.15  # rad/s² (smooth acceleration)
        self.last_angular_speed = 0.0  # For smooth transitions
        
        # Obstacle detection parameters
        self.obstacle_stop_distance = 0.5  # meters (50cm as requested)
        self.obstacle_detection_enabled = True  # ENABLED for obstacle avoidance
        
        # Gesture detection parameters (Task D)
        self.gesture_threshold = 80  # pixels - hand above shoulder (lower for easier detection)
        self.gesture_confirmation_frames = 2  # out of 6 frames (faster response)
        
        # Smooth acceleration/deceleration parameters
        self.acceleration_rate = 0.04  # m/s per cycle (smooth start)
        self.deceleration_rate = 0.05  # m/s per cycle (smooth stop)
        self.is_accelerating = False
        self.is_decelerating = False
        self.target_speed = 0.0
        
        # Obstacle state
        self.obstacle_paused = False  # True when paused due to obstacle
        
        rospy.loginfo("Person Detection Node initialized successfully!")
        rospy.loginfo("Waiting for camera data...")
        rospy.loginfo("Auto-lock enabled - will lock on first detected person")
        rospy.loginfo("Gesture control: Right hand = START, Left hand = STOP")

        
    def rgb_callback(self, msg):
        """Callback for RGB image from camera"""
        try:
            # No mirroring - use normal camera view
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting RGB image: {e}")
    
    def depth_callback(self, msg):
        """Callback for depth image from camera"""
        try:
            # No mirroring - use normal camera view
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {e}")

    
    def detect_person(self, image):
        """Detect persons in the image using YOLOv8"""
        try:
            results = self.model(
                image, 
                imgsz=320,  # Match the resize size in process_frame
                device=self.device,
                half=(self.device == "cuda"),
                classes=[0],  # Only detect persons (class 0)
                conf=0.45,  # Slightly lower for better detection
                verbose=False
            )
            
            detections = []
            for result in results:
                boxes = result.boxes
                if boxes is not None and len(boxes) > 0:
                    for box in boxes:
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        detections.append({
                            'bbox': [x1, y1, x2, y2],
                            'confidence': float(box.conf[0]),
                            'center': ((x1 + x2) // 2, (y1 + y2) // 2)
                        })
            
            return detections
        except Exception as e:
            rospy.logerr(f"Error in detect_person: {e}")
            return []
    
    def get_distance(self, center_x, center_y):
        """Get distance to object at given pixel coordinates using depth image"""
        if self.depth_image is None:
            return None
        
        height, width = self.depth_image.shape
        
        if 0 <= center_x < width and 0 <= center_y < height:
            depth_value = self.depth_image[center_y, center_x]
            
            if depth_value > 0:
                distance_meters = depth_value / 1000.0
                return distance_meters
        
        return None
    
    def match_person_to_locked(self, detections):
        """Match current detections to locked person"""
        if not self.locked_person or not detections:
            return None
        
        best_match = None
        best_score = 0
        
        locked_center = self.locked_person['center']
        locked_dist = self.locked_person.get('distance', 2.0)
        
        for det in detections:
            if 'distance' not in det or det['distance'] is None:
                continue
            
            # Distance similarity (closer = better)
            dist_diff = abs(det['distance'] - locked_dist)
            dist_score = max(0, 1 - (dist_diff / self.lock_distance_threshold))
            
            # Position similarity (closer = better)
            pos_diff = ((det['center'][0] - locked_center[0])**2 + 
                       (det['center'][1] - locked_center[1])**2)**0.5
            pos_score = max(0, 1 - (pos_diff / self.lock_position_threshold))
            
            # Combined score - prioritize position over distance for stability
            total_score = (dist_score * 0.4) + (pos_score * 0.6)
            
            # Use stricter threshold to avoid switching persons
            if total_score > best_score and total_score > self.lock_match_threshold:
                best_score = total_score
                best_match = det
        
        # Log matching info
        if best_match:
            rospy.logdebug_throttle(5, f"Match score: {best_score:.2f}")
        else:
            rospy.logdebug_throttle(5, f"No match found (best score: {best_score:.2f})")
        
        return best_match
    
    def find_closest_person(self, detections):
        """Find the closest person from all detections"""
        if not detections:
            return None
        
        valid_detections = []
        for det in detections:
            cx, cy = det['center']
            distance = self.get_distance(cx, cy)
            if distance is not None:
                det['distance'] = distance
                valid_detections.append(det)
        
        if not valid_detections:
            return None
        
        closest = min(valid_detections, key=lambda x: x['distance'])
        return closest
    
    def distance_controller(self, current_distance):
        """Task B: Calculate linear velocity based on distance to target"""
        if current_distance is None:
            return 0.0
        
        # PD controller
        error = current_distance - self.target_distance
        derivative = error - self.last_distance_error
        self.last_distance_error = error
        
        # Calculate speed
        speed = (self.Kp_linear * error) + (self.Kd_linear * derivative)
        
        # Apply limits
        speed = max(self.min_linear_speed, min(self.max_linear_speed, speed))
        
        # Emergency stop
        if current_distance < self.emergency_stop_distance:
            speed = -0.05  # Slow backup
        
        return speed
    
    def centering_controller(self, person_center_x):
        """Task C: Calculate angular velocity to keep person centered"""
        if person_center_x is None or self.rgb_image is None:
            return 0.0
        
        if self.rgb_image is None:
            return 0.0
            
        image_center = self.rgb_image.shape[1] / 2
        error = person_center_x - image_center
        
        # Add deadband to prevent constant small adjustments
        deadband = 50  # pixels (increased for more stability)
        if abs(error) < deadband:
            # Gradually slow down to zero
            self.last_angular_speed *= 0.7
            if abs(self.last_angular_speed) < 0.01:
                self.last_angular_speed = 0.0
            return self.last_angular_speed
        
        # Proportional control
        angular = -self.Kp_angular * error
        
        # Smooth acceleration (limit rate of change)
        angular_change = angular - self.last_angular_speed
        max_change = self.angular_acceleration_limit * 0.1  # Assuming 10Hz update
        if abs(angular_change) > max_change:
            angular = self.last_angular_speed + max_change * (1 if angular_change > 0 else -1)
        
        # Apply limits
        angular = max(-self.max_angular_speed, min(self.max_angular_speed, angular))
        
        # Store for next iteration
        self.last_angular_speed = angular
        
        return angular
    
    def detect_obstacles(self):
        """Task C: Detect obstacles in front using depth image"""
        # Obstacle detection disabled
        if not self.obstacle_detection_enabled:
            return False
            
        if self.depth_image is None:
            return False
        
        height, width = self.depth_image.shape
        
        # Check front center region (60 degree arc)
        center_x = width // 2
        arc_width = width // 3
        x_start = max(0, center_x - arc_width // 2)
        x_end = min(width, center_x + arc_width // 2)
        y_start = height // 2
        y_end = height
        
        # Get minimum distance in front region
        front_region = self.depth_image[y_start:y_end, x_start:x_end]
        valid_depths = front_region[front_region > 0]
        
        if len(valid_depths) > 0:
            min_distance = np.min(valid_depths) / 1000.0  # Convert to meters
            return min_distance < self.obstacle_stop_distance
        
        return False
    
    def detect_pose_and_gesture(self, image, bbox):
        """Task D: Detect pose and recognize hand gestures"""
        try:
            # Run pose detection on the person's bounding box
            x1, y1, x2, y2 = bbox
            
            # Validate and clip bbox to image bounds
            h, w = image.shape[:2]
            x1 = max(0, min(x1, w-1))
            y1 = max(0, min(y1, h-1))
            x2 = max(x1+1, min(x2, w))
            y2 = max(y1+1, min(y2, h))
            
            person_crop = image[y1:y2, x1:x2]
            
            if person_crop.size == 0 or person_crop.shape[0] < 20 or person_crop.shape[1] < 20:
                rospy.logdebug_throttle(10, "Person crop too small for pose detection")
                return None, 'NONE'
            
            # Detect pose with lower confidence for better detection
            results = self.pose_model(person_crop, verbose=False, conf=0.25)
            
            if len(results) == 0:
                rospy.logdebug_throttle(10, "No pose results")
                return None, 'NONE'
                
            if results[0].keypoints is None or len(results[0].keypoints) == 0:
                rospy.logdebug_throttle(10, "No keypoints in results")
                return None, 'NONE'
            
            keypoints = results[0].keypoints.xy[0].cpu().numpy()
            
            # Check if we got enough keypoints
            if len(keypoints) < 11:
                rospy.logdebug_throttle(10, f"Not enough keypoints: {len(keypoints)}")
                return None, 'NONE'
            
            # Adjust keypoints to original image coordinates
            keypoints[:, 0] += x1
            keypoints[:, 1] += y1
            
            # Detect gesture
            gesture = self.recognize_gesture(keypoints)
            
            rospy.loginfo_throttle(10, f"✓ Pose detected! Gesture: {gesture}")
            
            return keypoints, gesture
            
        except Exception as e:
            rospy.logerr_throttle(10, f"Pose detection error: {e}")
            return None, 'NONE'
    
    def recognize_gesture(self, keypoints):
        """Task D: Recognize hand gestures from keypoints"""
        # YOLO-Pose keypoint indices:
        # 5: left shoulder, 6: right shoulder
        # 7: left elbow, 8: right elbow
        # 9: left wrist, 10: right wrist
        
        # Normal camera view (no mirroring)
        # Right hand in image = user's right hand
        # Left hand in image = user's left hand
        
        try:
            left_shoulder = keypoints[5]
            right_shoulder = keypoints[6]
            left_wrist = keypoints[9]
            right_wrist = keypoints[10]
            
            # Check if keypoints are valid (not zero)
            if np.all(left_shoulder == 0) or np.all(right_shoulder == 0):
                return 'NONE'
            
            # Right hand raised = START following
            if np.all(right_wrist != 0):
                if right_wrist[1] < right_shoulder[1] - self.gesture_threshold:
                    return 'START'
            
            # Left hand raised = STOP following
            if np.all(left_wrist != 0):
                if left_wrist[1] < left_shoulder[1] - self.gesture_threshold:
                    return 'STOP'
            
            return 'NONE'
            
        except Exception as e:
            return 'NONE'
    
    def update_gesture_state(self, gesture):
        """Task D: Update gesture state with 6-frame buffer for stability"""
        # Add current gesture to buffer
        self.gesture_buffer.append(gesture)
        
        # Count occurrences in buffer
        gesture_counts = Counter(self.gesture_buffer)
        
        # Confirm gesture if detected in 2+ of last 6 frames
        if gesture_counts.get('START', 0) >= self.gesture_confirmation_frames:
            if self.following_state != 'FOLLOWING':
                self.following_state = 'FOLLOWING'
                self.is_accelerating = True  # Initiate smooth acceleration
                self.obstacle_paused = False  # Clear any obstacle pause
                rospy.loginfo("✓ Gesture: START FOLLOWING (smooth acceleration)")
        elif gesture_counts.get('STOP', 0) >= self.gesture_confirmation_frames:
            if self.following_state != 'IDLE':
                self.following_state = 'IDLE'
                self.obstacle_paused = False  # Clear obstacle pause
                self.stop_robot(smooth=True)  # Smooth deceleration
                rospy.loginfo("✓ Gesture: STOP FOLLOWING (smooth stop)")
    
    
    def stop_robot(self, smooth=False):
        """Stop the robot - can be instant or smooth deceleration"""
        if smooth:
            # Initiate smooth deceleration
            self.is_decelerating = True
            self.target_stop_speed = 0.0
            rospy.loginfo("Initiating smooth deceleration...")
        else:
            # Instant stop
            cmd = Twist()
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.cmd_vel_pub.publish(cmd)
            self.current_linear_speed = 0.0
            self.current_angular_speed = 0.0
            self.is_decelerating = False
    
    def apply_deceleration(self):
        """Apply smooth deceleration to current speeds"""
        if not self.is_decelerating:
            return False
        
        # Gradually reduce linear speed
        if abs(self.current_linear_speed) > 0.01:
            if self.current_linear_speed > 0:
                self.current_linear_speed = max(0, self.current_linear_speed - self.deceleration_rate)
            else:
                self.current_linear_speed = min(0, self.current_linear_speed + self.deceleration_rate)
        else:
            self.current_linear_speed = 0.0
        
        # Gradually reduce angular speed
        if abs(self.current_angular_speed) > 0.01:
            self.current_angular_speed *= 0.7  # Reduce by 30% each cycle
        else:
            self.current_angular_speed = 0.0
        
        # Publish reduced speed
        cmd = Twist()
        cmd.linear.x = self.current_linear_speed
        cmd.angular.z = self.current_angular_speed
        self.cmd_vel_pub.publish(cmd)
        
        # Check if stopped
        if abs(self.current_linear_speed) < 0.01 and abs(self.current_angular_speed) < 0.01:
            self.is_decelerating = False
            self.stop_robot(smooth=False)  # Final instant stop to ensure zero
            rospy.loginfo("Deceleration complete - stopped")
            return True
        
        return False
    
    def apply_acceleration(self, target_linear, target_angular):
        """Apply smooth acceleration towards target speeds"""
        if not self.is_accelerating:
            # Not accelerating, use target speeds directly
            return target_linear, target_angular
        
        # Gradually increase linear speed towards target
        if abs(self.current_linear_speed - target_linear) > 0.01:
            if self.current_linear_speed < target_linear:
                self.current_linear_speed = min(target_linear, self.current_linear_speed + self.acceleration_rate)
            else:
                self.current_linear_speed = max(target_linear, self.current_linear_speed - self.acceleration_rate)
        else:
            self.current_linear_speed = target_linear
        
        # Gradually increase angular speed towards target
        if abs(self.current_angular_speed - target_angular) > 0.01:
            angular_diff = target_angular - self.current_angular_speed
            self.current_angular_speed += angular_diff * 0.3  # 30% of difference
        else:
            self.current_angular_speed = target_angular
        
        # Check if acceleration complete
        if abs(self.current_linear_speed - target_linear) < 0.01 and abs(self.current_angular_speed - target_angular) < 0.01:
            self.is_accelerating = False
            rospy.loginfo("Acceleration complete - at target speed")
        
        return self.current_linear_speed, self.current_angular_speed
    
    def reset_to_idle(self):
        """Reset system to IDLE state - used for obstacle detection"""
        rospy.logwarn("⚠ RESETTING TO IDLE MODE ⚠")
        
        # Stop robot smoothly
        self.stop_robot(smooth=True)
        
        # Clear locked person
        self.locked_person = None
        self.lock_enabled = False
        self.auto_lock = True
        
        # Reset following state
        self.following_state = 'IDLE'
        
        # Clear gesture buffer
        self.gesture_buffer.clear()
        
        rospy.loginfo("System reset complete - ready for new person")
    
    def draw_detections(self, image, detections, locked_person, keypoints=None, gesture='NONE'):
        """Draw bounding boxes, skeleton, and labels on the image"""
        output_image = image.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            confidence = det['confidence']
            
            # Check if this is the locked person
            is_locked = (locked_person is not None and 
                        det['bbox'] == locked_person['bbox'])
            
            if is_locked:
                # Locked person: Blue box, thick border
                color = (255, 0, 0)  # Blue in BGR
                thickness = 4
                label = f"LOCKED: {confidence:.2f}"
            else:
                # Other persons: Yellow box, thin border
                color = (0, 255, 255)  # Yellow in BGR
                thickness = 2
                label = f"Person: {confidence:.2f}"
            
            # Draw bounding box
            cv2.rectangle(output_image, (x1, y1), (x2, y2), color, thickness)
            
            # Add distance label
            if 'distance' in det:
                label += f" | {det['distance']:.2f}m"
            
            # Draw label
            cv2.putText(output_image, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            # Draw center point
            cx, cy = det['center']
            cv2.circle(output_image, (cx, cy), 5, color, -1)
            
            # Draw skeleton on locked person
            if is_locked and keypoints is not None:
                self.draw_skeleton(output_image, keypoints, gesture)
        
        # Add following state indicator
        if self.following_state == 'FOLLOWING':
            status_text = "FOLLOWING"
            status_color = (0, 255, 0)  # Green
        else:
            status_text = "IDLE (Waiting for gesture)"
            status_color = (0, 165, 255)  # Orange
        
        cv2.putText(output_image, status_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 1, status_color, 3)
        
        # Add gesture status
        if gesture != 'NONE':
            gesture_text = f"Gesture: {gesture}"
            cv2.putText(output_image, gesture_text, (10, 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        
        # Add keyboard controls
        controls = "L:Lock | U:Unlock | R:Reset | Gestures: Right=Start, Left=Stop"
        cv2.putText(output_image, controls, (10, output_image.shape[0] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        return output_image
    
    def draw_skeleton(self, image, keypoints, gesture):
        """Draw skeleton keypoints and connections"""
        # YOLO-Pose skeleton connections
        skeleton = [
            [5, 6],   # shoulders
            [5, 7],   # left shoulder to elbow
            [7, 9],   # left elbow to wrist
            [6, 8],   # right shoulder to elbow
            [8, 10],  # right elbow to wrist
        ]
        
        # Draw connections
        for connection in skeleton:
            pt1_idx, pt2_idx = connection
            pt1 = keypoints[pt1_idx]
            pt2 = keypoints[pt2_idx]
            
            if np.all(pt1 != 0) and np.all(pt2 != 0):
                cv2.line(image, tuple(pt1.astype(int)), tuple(pt2.astype(int)), 
                        (0, 255, 0), 2)
        
        # Draw keypoints
        for i, kp in enumerate(keypoints):
            if np.all(kp != 0):
                # Highlight raised hands (normal view - no mirroring)
                if gesture == 'START' and i == 10:  # Right wrist = START
                    color = (0, 255, 0)  # Green
                    radius = 8
                elif gesture == 'STOP' and i == 9:  # Left wrist = STOP
                    color = (0, 0, 255)  # Red
                    radius = 8
                else:
                    color = (255, 255, 0)  # Cyan for other points
                    radius = 5
                
                cv2.circle(image, tuple(kp.astype(int)), radius, color, -1)
    
    
    def process_frame(self):
        """Process a single frame for person detection"""
        if self.rgb_image is None:
            return

        # Always show the camera feed to reduce lag
        self.frame_count += 1
        
        # Initialize variables for this frame
        keypoints = None
        gesture = 'NONE'
        
        # Run detection every 2nd frame
        if self.frame_count % 2 == 0:
            # Get original image dimensions
            orig_h, orig_w = self.rgb_image.shape[:2]
            
            # Resize image for faster inference
            detect_size = 320
            small = cv2.resize(self.rgb_image, (detect_size, detect_size), interpolation=cv2.INTER_LINEAR)
            detections = self.detect_person(small)
            
            # Scale bounding boxes back to original image size
            scale_x = orig_w / detect_size
            scale_y = orig_h / detect_size
            
            for det in detections:
                x1, y1, x2, y2 = det['bbox']
                det['bbox'] = [
                    int(x1 * scale_x),
                    int(y1 * scale_y),
                    int(x2 * scale_x),
                    int(y2 * scale_y)
                ]
                det['center'] = (
                    (det['bbox'][0] + det['bbox'][2]) // 2,
                    (det['bbox'][1] + det['bbox'][3]) // 2
                )
            
            # Add distances to all detections
            for det in detections:
                cx, cy = det['center']
                distance = self.get_distance(cx, cy)
                if distance is not None:
                    det['distance'] = distance
            
            # Person locking logic (Task A)
            if self.locked_person is None:
                rospy.loginfo_throttle(5, f"[LOCK] Searching... Detections:{len(detections)} AutoLock:{self.auto_lock}")
                if self.auto_lock and detections:
                    valid_detections = [d for d in detections if 'distance' in d]
                    rospy.loginfo_throttle(3, f"[LOCK] Valid detections with distance: {len(valid_detections)}")
                    if valid_detections:
                        self.locked_person = min(valid_detections, key=lambda x: x['distance'])
                        self.locked_person['last_seen'] = self.frame_count
                        self.lock_enabled = True
                        rospy.loginfo(f"✓✓✓ LOCKED on person at {self.locked_person['distance']:.2f}m ✓✓✓")
            else:
                matched_person = self.match_person_to_locked(detections)
                
                if matched_person:
                    self.locked_person = matched_person
                    self.locked_person['last_seen'] = self.frame_count
                    rospy.loginfo_throttle(10, f"[LOCK] Tracking at {self.locked_person['distance']:.2f}m")
                else:
                    frames_since_seen = self.frame_count - self.locked_person.get('last_seen', 0)
                    if frames_since_seen > self.lock_timeout_frames:
                        rospy.logwarn("✗✗✗ LOCK LOST - timeout ✗✗✗")
                        self.locked_person = None
                        self.lock_enabled = False
                        self.stop_robot()
                    else:
                        rospy.logwarn_throttle(3, f"[LOCK] Searching... ({frames_since_seen}/{self.lock_timeout_frames})")
            
            # Gesture detection on locked person (Task D)
            if self.locked_person is not None:
                keypoints, gesture = self.detect_pose_and_gesture(
                    self.rgb_image, 
                    self.locked_person['bbox']
                )
                
                # Update gesture state with buffer
                self.update_gesture_state(gesture)
            
            # Robot navigation (Tasks B & C)
            if self.following_state == 'FOLLOWING' and self.locked_person is not None:
                # Check for obstacles
                obstacle_detected = self.detect_obstacles()
                
                if obstacle_detected:
                    # OBSTACLE DETECTED - Pause (not reset)
                    if not self.obstacle_paused:
                        rospy.logwarn("⚠ OBSTACLE <50cm - PAUSING (person still locked) ⚠")
                        self.obstacle_paused = True
                    
                    # Stop smoothly but keep person locked
                    if not self.is_decelerating:
                        self.stop_robot(smooth=True)
                else:
                    # No obstacle - resume or continue following
                    if self.obstacle_paused:
                        rospy.loginfo("✓ Obstacle cleared - RESUMING following")
                        self.obstacle_paused = False
                        self.is_accelerating = True  # Smooth resume
                    
                    # Calculate control commands
                    target_linear = self.distance_controller(self.locked_person.get('distance'))
                    target_angular = self.centering_controller(self.locked_person['center'][0])
                    
                    # Apply smooth acceleration if starting/resuming
                    if self.is_accelerating:
                        linear_speed, angular_speed = self.apply_acceleration(target_linear, target_angular)
                    else:
                        linear_speed = target_linear
                        angular_speed = target_angular
                    
                    # Publish velocity command
                    cmd = Twist()
                    cmd.linear.x = linear_speed
                    cmd.angular.z = angular_speed
                    
                    self.cmd_vel_pub.publish(cmd)
                    
                    self.current_linear_speed = linear_speed
                    self.current_angular_speed = angular_speed
                    
                    # Log navigation commands
                    rospy.loginfo_throttle(2, 
                        f"Following: linear={linear_speed:.2f} m/s, angular={angular_speed:.2f} rad/s, dist={self.locked_person.get('distance'):.2f}m")
            else:
                # Not following - apply deceleration if needed, otherwise stop
                if self.is_decelerating:
                    self.apply_deceleration()
                elif self.current_linear_speed != 0 or self.current_angular_speed != 0:
                    self.stop_robot(smooth=False)
            
            # Draw detections on original image
            annotated_image = self.draw_detections(
                self.rgb_image, 
                detections, 
                self.locked_person,
                keypoints,
                gesture
            )
            
            # Cache the annotated image
            self.last_annotated = annotated_image
        else:
            # Use cached annotated image or raw image
            annotated_image = getattr(self, 'last_annotated', self.rgb_image)
        
        # Display image in OpenCV window (every frame for smooth display)
        cv2.imshow("Person Detection & Tracking", annotated_image)
        
        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('l') or key == ord('L'):
            # Lock on closest person
            if self.frame_count % 2 == 0 and 'detections' in locals():
                valid_detections = [d for d in detections if 'distance' in d]
                if valid_detections:
                    self.locked_person = min(valid_detections, key=lambda x: x['distance'])
                    self.locked_person['last_seen'] = self.frame_count
                    self.lock_enabled = True
                    self.auto_lock = False
                    rospy.loginfo(f"Manually locked on person at {self.locked_person['distance']:.2f}m")
        
        elif key == ord('u') or key == ord('U'):
            # Unlock current person
            if self.locked_person:
                rospy.loginfo("Unlocked person - ready to lock new target")
                self.locked_person = None
                self.lock_enabled = False
                self.auto_lock = False
                self.stop_robot()
        
        elif key == ord('r') or key == ord('R'):
            # Reset and enable auto-lock
            rospy.loginfo("Reset - auto-lock enabled")
            self.locked_person = None
            self.lock_enabled = False
            self.auto_lock = True
            self.following_state = 'IDLE'
            self.gesture_buffer.clear()
            self.stop_robot()
        
        # Publish annotated image (only when we have detections)
        if self.frame_count % 2 == 0:
            try:
                detection_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
                self.detection_image_pub.publish(detection_msg)
            except Exception as e:
                rospy.logerr(f"Error publishing detection image: {e}")
    
    def run(self):
        """Main loop for person detection"""
        rate = rospy.Rate(30)  # 30 Hz for smooth display
        
        rospy.loginfo("Starting person detection loop...")
        
        while not rospy.is_shutdown():
            self.process_frame()
            rate.sleep()
        
        cv2.destroyAllWindows()

def main():
    try:
        node = PersonDetectionNode()
        node.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Person detection node terminated.")
    except Exception as e:
        rospy.logerr(f"Error in person detection node: {e}")
        import traceback
        traceback.print_exc()

if __name__ == '__main__':
    main()
