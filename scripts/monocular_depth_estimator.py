import rospy
from sensor_msgs.msg import CompressedImage
import cv2
import numpy as np

class MonocularDepthEstimatiorNode:
    def __init__(self):
        rospy.init_node('monocular_depth_estimator', anonymous=True)
        input_img_topic_name = '/uav1/vio/camera/image_raw/compressed'
        self.subscriber = rospy.Subscriber(input_img_topic_name, CompressedImage, self.callback)
        rospy.loginfo("Listening on topic: " + input_img_topic_name)
        rospy.loginfo("Node Started")

    def callback(self, msg):
        rospy.loginfo("Received an image!")
        
        # Convert compressed image to numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if image is not None:
            height, width, _ = image.shape
            rospy.loginfo(f"Image dimensions: {width}x{height}")
        else:
            rospy.logwarn("Failed to decode image")

if __name__ == '__main__':
    node_instance = MonocularDepthEstimatiorNode()
    rospy.spin()
