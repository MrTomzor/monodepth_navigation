#!/usr/bin/env python3

import rospy
import numpy as np
from cv_bridge import CvBridge

class CameraProcessor:
    def __init__(self, cv_bridge: CvBridge):
        self.cv_bridge = cv_bridge
        self.k_matrix = None
        self.image = None
        self.image_time = None
    
    def set_camera_k_info(self, msg):
        if self.k_matrix is None:
            self.k_matrix = msg.K
            self.k_matrix = np.array(self.k_matrix).reshape(3, 3)
            rospy.loginfo(f"Received camera info")
    
    def update_image(self, msg):
        self.image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        self.image_time = msg.header.stamp
        if self.image is None or self.image_time is None:
            rospy.logwarn("Failed to decode image")
            return
    
    def scale_camera_k(self, K: np.ndarray, img_hw, depth_hw):
        H_img, W_img = img_hw
        H_d,   W_d   = depth_hw
        sx = W_d / float(W_img)
        sy = H_d / float(H_img)
        Kd = K.copy()
        Kd[0, 0] *= sx 
        Kd[1, 1] *= sy
        Kd[0, 2] *= sx
        Kd[1, 2] *= sy
        return Kd, sx, sy  
    
    def publish_image(self, image, publisher, encoding="bgr8"):
        ros_msg = self.cv_bridge.cv2_to_imgmsg(image, encoding=encoding)
        publisher.publish(ros_msg)
 

  
if __name__ == '__main__':
    CameraProcessor()
  
  
