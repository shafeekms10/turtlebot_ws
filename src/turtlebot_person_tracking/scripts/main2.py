#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import torch
import os
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
from ultralytics import YOLO

class PersonFollowerNode:
    def __init__(self):
        rospy.init_node("person_follower_node")

        self.bridge = CvBridge()
        self.device = "cuda" if torch.cuda.is_available() else "cpu"

        # Load models
        base = os.path.dirname(__file__)
        self.det_model = YOLO(os.path.join(base, "../models/yolov8n.pt"))
        self.pose_model = YOLO(os.path.join(base, "../models/yolov8n-pose.pt"))

        rospy.loginfo("YOLOv8 detection + pose models loaded")

        # Subscribers
        rospy.Subscriber("/camera/color/image_raw", Image, self.rgb_cb, queue_size=1)
        rospy.Subscriber("/camera/aligned_depth_to_color/image_raw", Image, self.depth_cb, queue_size=1)

        # Publishers
        self.cmd_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.img_pub = rospy.Publisher("/person_follower/image", Image, queue_size=1)

        # State
        self.rgb = None
        self.depth = None
        self.locked = None
        self.follow_enabled = False
        self.frame_id = 0

        # Control params
        self.target_dist = 1.5
        self.kp_dist = 0.6
        self.kp_angle = 0.004
        self.max_lin = 0.35
        self.max_ang = 0.8

        rospy.loginfo("Person follower node ready")

    # ------------------------------------------------
    def rgb_cb(self, msg):
        self.rgb = cv2.flip(self.bridge.imgmsg_to_cv2(msg, "bgr8"), 1)

    def depth_cb(self, msg):
        self.depth = cv2.flip(self.bridge.imgmsg_to_cv2(msg, "16UC1"), 1)

    # ------------------------------------------------
    def depth_at(self, x, y):
        if self.depth is None:
            return None
        h, w = self.depth.shape
        if 0 <= x < w and 0 <= y < h:
            d = self.depth[y, x]
            if d > 0:
                return d / 1000.0
        return None

    # ------------------------------------------------
    def detect_person_pose(self):
        res = self.pose_model(self.rgb, conf=0.5, device=self.device, verbose=False)[0]
        if res.keypoints is None:
            return None

        kpts = res.keypoints.xy[0].cpu().numpy()
        confs = res.keypoints.conf[0].cpu().numpy()

        def kp(i):
            return kpts[i] if confs[i] > 0.4 else None

        return {
            "nose": kp(0),
            "l_sh": kp(5),
            "r_sh": kp(6),
            "l_wr": kp(9),
            "r_wr": kp(10),
            "hip": kp(11) or kp(12)
        }

    # ------------------------------------------------
    def check_gestures(self, pose):
        if pose["r_wr"] is not None and pose["r_sh"] is not None:
            if pose["r_wr"][1] < pose["r_sh"][1]:
                self.follow_enabled = True
                rospy.loginfo_throttle(2, "START gesture detected")

        if pose["l_wr"] is not None and pose["l_sh"] is not None:
            if pose["l_wr"][1] < pose["l_sh"][1]:
                self.follow_enabled = False
                rospy.loginfo_throttle(2, "STOP gesture detected")

    # ------------------------------------------------
    def control_robot(self, pose):
        cmd = Twist()

        if not self.follow_enabled or pose["hip"] is None:
            self.cmd_pub.publish(cmd)
            return

        hx, hy = int(pose["hip"][0]), int(pose["hip"][1])
        dist = self.depth_at(hx, hy)

        if dist is None:
            self.cmd_pub.publish(cmd)
            return

        # Distance control
        err = dist - self.target_dist
        cmd.linear.x = np.clip(self.kp_dist * err, -self.max_lin, self.max_lin)

        # Angle control
        img_center = self.rgb.shape[1] // 2
        cmd.angular.z = np.clip(-self.kp_angle * (hx - img_center), -self.max_ang, self.max_ang)

        # Obstacle emergency stop
        if dist < 0.6:
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

    # ------------------------------------------------
    def draw(self, pose):
        img = self.rgb.copy()

        if pose["hip"] is not None:
            cv2.circle(img, tuple(pose["hip"].astype(int)), 6, (255, 0, 0), -1)

        status = "FOLLOWING" if self.follow_enabled else "STOPPED"
        cv2.putText(img, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 255, 0) if self.follow_enabled else (0, 0, 255), 3)

        self.img_pub.publish(self.bridge.cv2_to_imgmsg(img, "bgr8"))
        cv2.imshow("Person Follower", img)
        cv2.waitKey(1)

    # ------------------------------------------------
    def run(self):
        rate = rospy.Rate(20)
        while not rospy.is_shutdown():
            if self.rgb is None:
                rate.sleep()
                continue

            pose = self.detect_person_pose()
            if pose:
                self.check_gestures(pose)
                self.control_robot(pose)
                self.draw(pose)

            rate.sleep()

# ------------------------------------------------
if __name__ == "__main__":
    try:
        PersonFollowerNode().run()
    except rospy.ROSInterruptException:
        pass
