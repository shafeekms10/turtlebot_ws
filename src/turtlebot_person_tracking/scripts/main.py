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
        
        # Publisher for robot control (future use)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
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
        self.lock_distance_threshold = 0.5  # meters
        self.lock_position_threshold = 150  # pixels
        self.lock_timeout_frames = 90  # 3 seconds at 30fps
        
        rospy.loginfo("Person Detection Node initialized successfully!")
        rospy.loginfo("Waiting for camera data...")
        rospy.loginfo("Auto-lock enabled - will lock on first detected person")

        
    def rgb_callback(self, msg):
        """Callback for RGB image from camera"""
        try:
            image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            # Mirror the image horizontally
            self.rgb_image = cv2.flip(image, 1)
        except Exception as e:
            rospy.logerr(f"Error converting RGB image: {e}")
    
    def depth_callback(self, msg):
        """Callback for depth image from camera"""
        try:
            depth = self.bridge.imgmsg_to_cv2(msg, "16UC1")
            # Mirror the depth image to match RGB
            self.depth_image = cv2.flip(depth, 1)
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {e}")

    
    def detect_person(self, image):
        """Detect persons in the image using YOLOv8"""
        results = self.model(
            image, 
            imgsz=416, 
            device=self.device,
            half=(self.device == "cuda"),
            classes=[0],  # Only detect persons (class 0)
            conf=0.5,
            verbose=False
        )
        
        detections = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = map(int, box.xyxy[0])
                detections.append({
                    'bbox': [x1, y1, x2, y2],
                    'confidence': float(box.conf[0]),
                    'center': ((x1 + x2) // 2, (y1 + y2) // 2)
                })
        
        return detections
    
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
        locked_dist = self.locked_person['distance']
        
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
            
            # Combined score
            total_score = (dist_score * 0.6) + (pos_score * 0.4)
            
            if total_score > best_score and total_score > 0.3:
                best_score = total_score
                best_match = det
        
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
    
    def draw_detections(self, image, detections, locked_person):
        """Draw bounding boxes and labels on the image"""
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
        
        # Add lock status indicator
        if locked_person:
            status_text = "LOCKED TARGET"
            cv2.putText(output_image, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 0, 0), 3)
        else:
            status_text = "NO LOCK - Searching..."
            cv2.putText(output_image, status_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 255), 2)
        
        # Add keyboard controls
        controls = "L:Lock | U:Unlock | R:Reset"
        cv2.putText(output_image, controls, (10, output_image.shape[0] - 20),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
        
        return output_image
    
    def process_frame(self):
        """Process a single frame for person detection"""
        if self.rgb_image is None:
            return

        # Always show the camera feed to reduce lag
        self.frame_count += 1
        
        # Run detection every 2nd frame instead of every 3rd
        if self.frame_count % 2 == 0:
            # Get original image dimensions
            orig_h, orig_w = self.rgb_image.shape[:2]
            
            # Resize image for faster inference (smaller = faster)
            detect_size = 320
            small = cv2.resize(self.rgb_image, (detect_size, detect_size), interpolation=cv2.INTER_LINEAR)
            detections = self.detect_person(small)
            
            # Scale bounding boxes back to original image size
            scale_x = orig_w / detect_size
            scale_y = orig_h / detect_size
            
            for det in detections:
                x1, y1, x2, y2 = det['bbox']
                # Scale coordinates back to original size
                det['bbox'] = [
                    int(x1 * scale_x),
                    int(y1 * scale_y),
                    int(x2 * scale_x),
                    int(y2 * scale_y)
                ]
                # Update center point as well
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
            
            # Person locking logic
            if self.locked_person is None:
                # No locked person - auto-lock on first detected person
                if self.auto_lock and detections:
                    valid_detections = [d for d in detections if 'distance' in d]
                    if valid_detections:
                        # Lock on closest person
                        self.locked_person = min(valid_detections, key=lambda x: x['distance'])
                        self.locked_person['last_seen'] = self.frame_count
                        self.lock_enabled = True
                        rospy.loginfo(f"Auto-locked on person at {self.locked_person['distance']:.2f}m")
            else:
                # Have locked person - try to find them in current frame
                matched_person = self.match_person_to_locked(detections)
                
                if matched_person:
                    # Update locked person with new position
                    self.locked_person = matched_person
                    self.locked_person['last_seen'] = self.frame_count
                    rospy.loginfo_throttle(2, 
                        f"Tracking locked person at {self.locked_person['distance']:.2f}m")
                else:
                    # Lost locked person
                    frames_since_seen = self.frame_count - self.locked_person.get('last_seen', 0)
                    if frames_since_seen > self.lock_timeout_frames:
                        rospy.logwarn("Locked person lost - timeout exceeded")
                        self.locked_person = None
                        self.lock_enabled = False
                    else:
                        rospy.logwarn_throttle(1, "Locked person not found, searching...")
            
            # Draw detections on original image
            annotated_image = self.draw_detections(
                self.rgb_image, 
                detections, 
                self.locked_person
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
            if self.frame_count % 2 == 0 and detections:
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
        
        elif key == ord('r') or key == ord('R'):
            # Reset and enable auto-lock
            rospy.loginfo("Reset - auto-lock enabled")
            self.locked_person = None
            self.lock_enabled = False
            self.auto_lock = True
        
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
