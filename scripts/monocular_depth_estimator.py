import rospy
from sensor_msgs.msg import CompressedImage, Image
import cv2
import numpy as np
from cv_bridge import CvBridge

from monodepth_navigation.midas_extension import MidasExtension


class MonocularDepthEstimatorNode:
    def __init__(self):
        rospy.init_node('monocular_depth_estimator', anonymous=True)
        input_img_topic_name = '/uav1/vio/camera/image_raw/compressed'
        self.subscriber = rospy.Subscriber(input_img_topic_name, CompressedImage, self.callback)
        rospy.loginfo("Listening on topic: " + input_img_topic_name)
        rospy.loginfo("Node Started")


        self.pub_rgb = rospy.Publisher('/midas_debug/rgb_view', Image, queue_size=1)
        self.pub_depth = rospy.Publisher('/midas_debug/depth_view', Image, queue_size=1)
        
        self.bridge = CvBridge()

        self.midas = MidasExtension(model_type="MiDaS_small")

    def callback(self, msg):
        rospy.loginfo("Received an image!")
        
        # Convert compressed image to numpy array
        np_arr = np.frombuffer(msg.data, np.uint8)
        image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        
        if image is not None:
            height, width, _ = image.shape
            rospy.loginfo(f"Image dimensions: {width}x{height}")
            depth_color = self.midas.run(image)
            
            ros_rgb = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")
            self.pub_rgb.publish(ros_rgb)

            ros_depth = self.bridge.cv2_to_imgmsg(depth_color, encoding="bgr8")
            self.pub_depth.publish(ros_depth)
        else:
            rospy.logwarn("Failed to decode image")

if __name__ == '__main__':
    node_instance = MonocularDepthEstimatorNode()
    rospy.spin()
