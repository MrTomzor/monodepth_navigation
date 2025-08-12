import rospy
from sensor_msgs.msg import Image, PointCloud2, CameraInfo
import tf2_ros
import cv2
import numpy as np
from cv_bridge import CvBridge
import time
from collections import deque
from scipy.interpolate import griddata
from monodepth_navigation.midas_extension import MidasExtension
from monodepth_navigation.pointcloud_processor import PointCloudProcessor
from monodepth_navigation.scale_estimator import *
from monodepth_navigation.camera_processor import CameraProcessor

class MonocularDepthEstimatorNode:
    def __init__(self):
        rospy.init_node('monocular_depth_estimator', anonymous=True)
        self.init_params()
        self.init_tf()
        self.init_state()
        self.init_publishers()
        self.init_subscribers()
        self.init_timer()
        rospy.loginfo("Node Started")
        
        
    def init_state(self):
        self.midas = MidasExtension(model_type="DPT_Large")
        #self.midas = MidasExtension(model_type="DPT_Hybrid")
        #self.midas = MidasExtension(model_type="MiDaS_small")

        #self.depth_map = None
        #self.scaled_depth_map = None
        self.scale = 1
        self.cloud_buffer = deque(maxlen=30)
        self.pointcloud = PointCloudProcessor(self.tf_buffer)
        self.camera = CameraProcessor(self.bridge)

    def init_params(self):
        #self.output_rgb_image_topic_name = rospy.get_param("output_openVins_image_topic")
        self.output_depth_map_topic_name = rospy.get_param("output_depth_map_topic")

        self.output_scaled_depth_map_topic_name_map = rospy.get_param("output_scaled_depth_map_topic_map")
        self.output_scaled_depth_map_topic_name_value = rospy.get_param("output_scaled_depth_map_topic_value")

        self.output_pointcloud_topic_map_name = rospy.get_param("output_pointcloud_topic_map")
        self.output_pointcloud_topic_value_name = rospy.get_param("output_pointcloud_topic_value")


        self.input_pointclouds_topic_name = rospy.get_param("input_pointcloud_topic", "/uav1/lidar/points")
        self.input_img_topic_name = rospy.get_param("input_img_topic", "/uav1/stereo/left/image_raw")
        self.input_rgbd_color_cam_info_topic_name = rospy.get_param("input_camera_info_topic", "/uav1/stereo/left/camera_info")
        self.camera_frame = rospy.get_param("camera_frame", "uav1/stereo_left")

    def init_tf(self):
        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

    def init_publishers(self):
        #self.pub_rgb = rospy.Publisher(self.output_rgb_image_topic_name, Image, queue_size=1)
        self.pub_depth = rospy.Publisher(self.output_depth_map_topic_name, Image, queue_size=1)

        self.pub_scaled_depth_map = rospy.Publisher(self.output_scaled_depth_map_topic_name_map, Image, queue_size=1)
        self.pub_scaled_depth_value = rospy.Publisher(self.output_scaled_depth_map_topic_name_value, Image, queue_size=1)

        self.pub_pointcloud_map = rospy.Publisher(self.output_pointcloud_topic_map_name, PointCloud2, queue_size=1)
        self.pub_pointcloud_value = rospy.Publisher(self.output_pointcloud_topic_value_name, PointCloud2, queue_size=1)

    def init_subscribers(self):
        rospy.Subscriber(self.input_img_topic_name, Image, self.image_callback, queue_size = 1)
        rospy.Subscriber(self.input_pointclouds_topic_name, PointCloud2, self.pointcloud_callback)
        rospy.Subscriber(self.input_rgbd_color_cam_info_topic_name, CameraInfo, self.camera_info_callback)
    
    def init_timer(self):
        self.timer = rospy.Timer(rospy.Duration(0.01), self.compute_midas_pointcloud)

    def image_callback(self, image_msg):
        self.camera.update_image(image_msg)

    def camera_info_callback(self, cam_info_msg):
        self.camera.set_camera_k_info(cam_info_msg)

    def pointcloud_callback(self, cloud_msg):
        if self.camera.k_matrix is None:
            rospy.logwarn("Camera intrinsic matrix not initialized yet.")
            return
        
        raw_pointlcoud = self.pointcloud.read_pointcloud(cloud_msg)
        transformed_pointcloud = self.pointcloud.change_points_frame(self.camera_frame, cloud_msg.header.frame_id, self.camera_frame, raw_pointlcoud, cloud_msg.header.stamp)
        if transformed_pointcloud is None or len(transformed_pointcloud) == 0:
            rospy.logwarn("There is no pointcloud in frame")
            return
        
        pointcloud_2d = self.pointcloud.project_points_3d_to_2d(transformed_pointcloud, self.camera.k_matrix)
        self.cloud_buffer.append((cloud_msg.header.stamp, pointcloud_2d))
        
        
    def compute_midas_pointcloud(self, event):
        current_image = self.camera.image
        current_time = self.camera.image_time

        if current_image is None:
            rospy.logwarn("Waiting for Image")
            return
        if not self.cloud_buffer:
            rospy.logwarn(f"Waiting for {self.input_pointclouds_topic_name} pointcloud data")
            return

        depth_color, raw_depth = self.midas.run(current_image)
        self.camera.publish_image(depth_color, self.pub_depth)  # MiDas depth map before scaling
        depth_map = 1.0 / (raw_depth + 1e-12)  # The closest object is 0, the farthest is 1
        current_pointcloud = self.find_pointcloud(current_time)

        
        scale_value = get_scale_value(current_pointcloud, depth_map)
        scale_map = get_scale_map(current_pointcloud, depth_map)

        scaled_depth_map_by_map = depth_map * scale_map
        scaled_depth_map_by_value = depth_map * scale_value
        
        cloud_msg_map = self.pointcloud.create_cloud_msg(scaled_depth_map_by_map, current_time, self.camera.k_matrix, self.camera_frame)
        self.pointcloud.publish_pointcloud(self.pub_pointcloud_map, cloud_msg_map)
        cloud_msg_value = self.pointcloud.create_cloud_msg(scaled_depth_map_by_value, current_time, self.camera.k_matrix, self.camera_frame)
        self.pointcloud.publish_pointcloud(self.pub_pointcloud_value, cloud_msg_value)

        # MiDas depth map after scaling
        inverse_depth_map_by_map = 1.0 / (scaled_depth_map_by_map + 1e-9)
        colorized_depth_image_map = self.colorize_inverse_depth(inverse_depth_map_by_map)
        self.camera.publish_image(colorized_depth_image_map, self.pub_scaled_depth_map) 
        inverse_depth_map_by_value = 1.0 / (scaled_depth_map_by_value + 1e-9)
        colorized_depth_image_value = self.colorize_inverse_depth(inverse_depth_map_by_value)
        self.camera.publish_image(colorized_depth_image_value, self.pub_scaled_depth_value) 
    


    def find_pointcloud(self, image_time):
        if not self.cloud_buffer:
            rospy.logwarn("No pointcloud in cloud_buffer")
            return None

        closest = min(
            self.cloud_buffer, 
            key=lambda x: abs((x[0] - image_time).to_sec())
        )
        rospy.loginfo(f"nejstarsi {self.cloud_buffer[0][0].to_sec()}")
        rospy.loginfo(f"nejnovejsi {self.cloud_buffer[-1][0].to_sec()}")
        rospy.loginfo(image_time.to_sec())

        _, pointcloud = closest
        return pointcloud 


    def colorize_inverse_depth(self, inverse_depth_map):
        inverse_depth_map = np.nan_to_num(inverse_depth_map, nan=0.0, posinf=0.0, neginf=0.0)
        normalized = cv2.normalize(inverse_depth_map, None, 0, 255, cv2.NORM_MINMAX)
        normalized = normalized.astype(np.uint8)
        colorized = cv2.applyColorMap(normalized, cv2.COLORMAP_MAGMA)
        return colorized


                 


        

if __name__ == '__main__':
    node_instance = MonocularDepthEstimatorNode()
    rospy.spin()
