#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import torch
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO

class PersonDetectionNode:
    def __init__(self):
        rospy.init_node('person_detection_node', anonymous=True)
        self.device = "cuda" if torch.cuda.is_available() else "cpu"
        self.bridge = CvBridge()
        
        rospy.loginfo("Loading YOLOv8 model...")
        self.model = YOLO('/home/shafeek/models/yolov8n.pt')
        rospy.loginfo("YOLOv8 model loaded successfully!")
        
        self.rgb_image = None
        self.depth_image = None
        self.latest_detection = None
        self.frame_count = 0
        
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
        
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        
        self.detection_image_pub = rospy.Publisher(
            '/person_detection/image', 
            Image, 
            queue_size=10
        )
        
        self.confidence_threshold = 0.5
        self.person_class_id = 0
        
        rospy.loginfo("Person Detection Node initialized successfully!")
        rospy.loginfo("Waiting for camera data...")
        
    def rgb_callback(self, msg):
        try:
            self.rgb_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            rospy.logerr(f"Error converting RGB image: {e}")
    
    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, "16UC1")
        except Exception as e:
            rospy.logerr(f"Error converting depth image: {e}")
    
    def detect_person(self, image):
        results = self.model(
            image, 
            imgsz=416, 
            device=self.device,
            half = (self.device == "cuda"),
            classes =[0],conf=0.5 ,
            verbose=False)
        
        detections = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                # class_id = int(box.cls[0])
                # confidence = float(box.conf[0])
                
                # if class_id == self.person_class_id and confidence >= self.confidence_threshold:
                #     x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                #     detections.append({
                #         'bbox': [int(x1), int(y1), int(x2), int(y2)],
                #         'confidence': confidence,
                #         'center': (int((x1 + x2) / 2), int((y1 + y2) / 2))
                #     })
                x1, y1, x2, y2 = map(int,box.xyxy[0])
                detections.append({
                    'bbox': [x1, y1, x2, y2],
                    'confidence': float(box.conf[0]),
                    'center': (((x1 + x2) // 2), ((y1 + y2) // 2))
                })
        
        return detections
    
    def get_distance(self, center_x, center_y):
        if self.depth_image is None:
            return None
        
        height, width = self.depth_image.shape
        
        if 0 <= center_x < width and 0 <= center_y < height:
            depth_value = self.depth_image[center_y, center_x]
            
            if depth_value > 0:
                distance_meters = depth_value / 1000.0
                return distance_meters
        
        return None
    
    def find_closest_person(self, detections):
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
    
    def draw_detections(self, image, detections, closest_person):
        output_image = image.copy()
        
        for det in detections:
            x1, y1, x2, y2 = det['bbox']
            confidence = det['confidence']
            
            is_closest = (closest_person is not None and 
                         det['bbox'] == closest_person['bbox'])
            
            color = (0, 255, 0) if is_closest else (255, 0, 0)
            thickness = 3 if is_closest else 2
            
            cv2.rectangle(output_image, (x1, y1), (x2, y2), color, thickness)
            
            label = f"Person: {confidence:.2f}"
            if 'distance' in det:
                label += f" | {det['distance']:.2f}m"
            
            cv2.putText(output_image, label, (x1, y1 - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
            
            cx, cy = det['center']
            cv2.circle(output_image, (cx, cy), 5, color, -1)
        
        if closest_person:
            cv2.putText(output_image, "TRACKING TARGET", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        return output_image
    
    def process_frame(self):
        if self.rgb_image is None:
            return

        self.frame_count += 1
        if self.frame_count % 3 != 0:
            return
        
        # detections = self.detect_person(self.rgb_image)
        small = cv2.resize(self.rgb_image, (416, 416),interpolation=cv2.INTER_LINEAR)
        detections = self.detect_person(small)
        
        closest_person = self.find_closest_person(detections)
        
        self.latest_detection = closest_person
        
        annotated_image = self.draw_detections(
            self.rgb_image, 
            detections, 
            closest_person
        )
        
        # cv2.imshow("Person Detection", annotated_image)
        # cv2.waitKey(1)
        
        try:
            detection_msg = self.bridge.cv2_to_imgmsg(annotated_image, "bgr8")
            self.detection_image_pub.publish(detection_msg)
        except Exception as e:
            rospy.logerr(f"Error publishing detection image: {e}")
        
        if closest_person:
            rospy.loginfo_throttle(1, 
                f"Tracking person at distance: {closest_person['distance']:.2f}m")
    
    def run(self):
        rate = rospy.Rate(5)
        
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
