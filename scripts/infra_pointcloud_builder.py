#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Image, CameraInfo, PointCloud2, PointField
from std_msgs.msg import Header
import sensor_msgs.point_cloud2 as pc2
import numpy as np
from cv_bridge import CvBridge
import tf2_ros
from monodepth_navigation.pointcloud_processor import PointCloudProcessor
from monodepth_navigation.camera_processor import CameraProcessor

class InfraPointcloudBuilder:
    def __init__(self):
        rospy.init_node('infra_pointcloud_builder', anonymous=True)
        self.init_params()
        self.init_tf()
        self.init_state()
        self.init_publishers()
        self.init_subscribers()
        rospy.loginfo("Node Started")


    def init_params(self):
        self.input_depth_topic = rospy.get_param("input_depth_topic","/uav1/rgbd/aligned_depth_to_color/image_raw")
        self.input_caminfo_topic = rospy.get_param("input_camera_info_topic","/uav1/rgbd/color/camera_info")
        self.output_cloud_topic = rospy.get_param("output_pointcloud_topic", "/infra/pointcloud")
        self.camera_frame = rospy.get_param("camera_frame", "uav1/rgbd/color_optical")

    def init_tf(self):
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def init_state(self):
        self.bridge = CvBridge()
        self.pointcloud = PointCloudProcessor(self.tf_buffer)
        self.camera = CameraProcessor(self.bridge)

    def init_publishers(self):
        self.pub_pointcloud = rospy.Publisher(self.output_cloud_topic, PointCloud2, queue_size=1)

    def init_subscribers(self):
        rospy.Subscriber(self.input_depth_topic, Image, self.callback, queue_size=1)
        rospy.Subscriber(self.input_caminfo_topic, CameraInfo, self.camera_info_callback, queue_size=1)


    def camera_info_callback(self, msg):
        self.camera.set_camera_k_info(msg)
   

    def callback(self, msg):
        if self.camera.k_matrix is None:
            rospy.logwarn("Camera intrinsics not received yet!")
            return
       
        msg_time = msg.header.stamp
        depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')

        if depth_image.dtype == np.uint16:
            depth_meters = depth_image.astype(np.float32) / 1000.0
        elif depth_image.dtype == np.float32:
            depth_meters = depth_image
        else:
            rospy.logerr("Unsupported depth image format")
            return

       
        cloud_msg = self.pointcloud.create_cloud_msg(depth_meters, msg_time, self.camera.k_matrix, self.camera_frame)
        self.pointcloud.publish_pointcloud(self.pub_pointcloud, cloud_msg)
     


  


if __name__ == '__main__':
    InfraPointcloudBuilder()
    rospy.spin()
