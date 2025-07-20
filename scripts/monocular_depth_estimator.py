# ROS core
import rospy

# ROS messages
from sensor_msgs.msg import Image, CompressedImage, PointCloud2, PointField, CameraInfo
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header

# ROS utilities
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_geometry_msgs

# Computer vision and math
import cv2
import numpy as np
from cv_bridge import CvBridge
import random

import time



import tf.transformations as tft  

# MiDaS depth estimation
from monodepth_navigation.midas_extension import MidasExtension  # type: ignore


class MonocularDepthEstimatorNode:
    def __init__(self):
        rospy.init_node('monocular_depth_estimator', anonymous=True)
        rospy.loginfo("Node Started")
        
        self.midas = MidasExtension(model_type="MiDaS_small")
        #self.midas = MidasExtension(model_type="DPT_Large")
        #self.midas = MidasExtension(model_type="DPT_Hybrid")

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.camera_info_received = False
        self.camera_K = None 
        self.image = None  
        self.depth_map = None
        self.scaled_depth_map = None
        self.received_openvins_points = False

        self.scale = 1

        
        output_rgb_image_topic_name = '/midas/rgb_OpenVins_view'
        output_depth_map_topic_name = '/midas/depth_view'
        output_scaled_depth_map_topic_name = '/midas/scaled_depth_view'
        output_pointcloud_topic_name = '/midas/pointcloud'
        
        self.pub_rgb = rospy.Publisher(output_rgb_image_topic_name, Image, queue_size=1)
        self.pub_depth = rospy.Publisher(output_depth_map_topic_name, Image, queue_size=1)
        self.pub_scaled_depth = rospy.Publisher(output_scaled_depth_map_topic_name, Image, queue_size=1)
        self.pub_pointcloud = rospy.Publisher(output_pointcloud_topic_name, PointCloud2, queue_size=1)


        #input_img_topic_name = '/uav1/rgbd/color/image_raw'
        #input_pointclouds_topic_name = '/ov_msckf/points_slam'
        #input_rgbd_color_cam_info_topic_name = '/uav1/rgbd/color/camera_info'

        input_pointclouds_topic_name = rospy.get_param("input_pointcloud_topic", "/uav1/lidar/points")
        input_img_topic_name = rospy.get_param("input_img_topic", "/uav1/stereo/left/image_raw")
        input_rgbd_color_cam_info_topic_name = rospy.get_param("input_camera_info_topic", "/uav1/stereo/left/camera_info")

        self.camera_frame = rospy.get_param("camera_frame", "uav1/stereo_left")

        rospy.Subscriber(input_img_topic_name, Image, self.callback, queue_size = 1)
        rospy.Subscriber(input_pointclouds_topic_name, PointCloud2, self.pointcloud_callback)
        rospy.Subscriber(input_rgbd_color_cam_info_topic_name, CameraInfo, self.camera_info_callback)

        rospy.loginfo("Listening on topics: " + input_img_topic_name + " " + input_pointclouds_topic_name + " " + input_rgbd_color_cam_info_topic_name)


    def callback(self, image_msg):
        if not self.received_openvins_points:
            rospy.logwarn("Waiting for OpenVINS pointcloud data")

        self.image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        if self.image is None:
            rospy.logwarn("Failed to decode image")
            return
        
        depth_color, raw_depth = self.midas.run(self.image)
        self.publish_image(depth_color, self.pub_depth)  # MiDas depth map before scaling
    
        self.depth_map = 1.0 / (raw_depth + 1e-12)      # The closest object is 0, the farthest is 1
        self.scaled_depth_map = self.depth_map * self.scale

        self.create_pointcloud(self.scaled_depth_map)

        inverse_depth_map = 1.0 / (self.scaled_depth_map + 1e-9)
        colorized_depth_image = self.colorize_inverse_depth(inverse_depth_map)
        self.publish_image(colorized_depth_image, self.pub_scaled_depth) # MiDas depth map after scaling

        

    def colorize_inverse_depth(self, inverse_depth_map):
        inverse_depth_map = np.nan_to_num(inverse_depth_map, nan=0.0, posinf=0.0, neginf=0.0)
        normalized = cv2.normalize(inverse_depth_map, None, 0, 255, cv2.NORM_MINMAX)
        normalized = normalized.astype(np.uint8)
        colorized = cv2.applyColorMap(normalized, cv2.COLORMAP_MAGMA)

        return colorized

    
    def publish_image(self, cv_image, publisher, encoding="bgr8"):
        ros_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding=encoding)
        publisher.publish(ros_msg)


    
    def camera_info_callback(self, cam_info_msg):
        if not self.camera_info_received:
            self.camera_K = cam_info_msg.K 
            self.camera_info_received = True 
            self.camera_K = np.array(self.camera_K).reshape(3, 3)
            rospy.loginfo(f"Received camera info")
            
    
    def pointcloud_callback(self, cloud_msg):
        self.received_openvins_points = True

        if not self.camera_info_received or self.image is None:
            rospy.logwarn("Camera info not yet received or Image is None.")
            return
        
        if self.depth_map is None:
            rospy.logwarn("Depth map from MiDas not yet received, skipping pointcloud scaling.")
            return
        
        pointcloud = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True))

        transformed_points = self.change_points_frame(cloud_msg.header.frame_id, self.camera_frame, pointcloud, cloud_msg.header.stamp)
        #transformed_points = self.change_points_frame(cloud_msg.header.frame_id , 'uav1/rgbd/color_optical', pointcloud, cloud_msg.header.stamp)
        if not transformed_points:
            rospy.logwarn("All points_frame transforms failed — no points converted")
            return
        
        pointcloud_2d = self.project_points_3d_to_2d(transformed_points)

        self.vizualize_2d_pointcloud(pointcloud_2d)

        self.update_scale(pointcloud_2d)
        #rospy.loginfo(f"Received {len(pointcloud)} 3D cloudpoints. \n Transformed {len(transformed_points)} points. \n Projected {len(pointcloud_2d)} points.\n\n\n")

    def update_scale(self, pointcloud_2d):
        pairs = []

        for (u, v, d) in pointcloud_2d:
            u = int(u)
            v = int(v)
            if (0 <= u < self.depth_map.shape[1]) and (0 <= v < self.depth_map.shape[0]):
                midas_depth = self.depth_map[v, u]
                if midas_depth > 0 and np.isfinite(midas_depth):
                    pairs.append((midas_depth, d))

        if len(pairs) == 0:
            return

        N = 3
        pairs.sort(key=lambda x: x[1])
        closest_pairs = pairs[:N] 

        abstract_depths = np.array([p[0] for p in closest_pairs])
        real_depths = np.array([p[1] for p in closest_pairs])

        new_scale = np.median(real_depths / abstract_depths)
        if np.isfinite(new_scale) and 0.01 < new_scale < 1000.0:
            self.scale = new_scale
            #rospy.loginfo(f"Updated scale from {len(closest_pairs)} nearest points: {self.scale:.4f}. Total OpenVINS points in frame: {len(pairs)}")

                    
            
    def vizualize_2d_pointcloud(self, pointcloud_2d):
        image_copy = self.image.copy()
        for (u, v, d) in pointcloud_2d:
            u = int(u)
            v = int(v)
            d = int(d)

            cv2.circle(image_copy, (u, v), 10, (0, 255, 0), -1)
            self.publish_image(image_copy, self.pub_rgb)
     
    

    def change_points_frame(self, source_frame, target_frame, points, cloud_time):
        try:
            tf_msg = self.tf_buffer.lookup_transform(target_frame, source_frame, rospy.Time(0), rospy.Duration(0.1))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException) as e:
            rospy.logwarn(f"TF lookup failed: {e}")
            return []     

        t = tf_msg.transform.translation
        q = tf_msg.transform.rotation
        T = tft.quaternion_matrix([q.x, q.y, q.z, q.w])   
        T[0:3, 3] = [t.x, t.y, t.z]     

        transformed = []
        for x, y, z in points:
            vec = np.array([x, y, z, 1.0])   
            x2, y2, z2, _ = T.dot(vec)
            if z2 > 0:           
                transformed.append((x2, y2, z2))
        return transformed

    
    
    def project_points_3d_to_2d(self, pointclouds):
        pixel_coords = []
        for x, y, z in pointclouds:
            if z <= 0:  
                continue
            
            K = self.camera_K
            u = ((K[0, 0] * x) / z) + K[0, 2]
            v = ((K[1, 1] * y) / z) + K[1, 2]

            pixel_coords.append((int(round(u)), int(round(v)), z))
        return pixel_coords

    
    def create_pointcloud(self, frame):
        height, width = frame.shape
        points = []
        camera_K_inv = np.linalg.inv(self.camera_K)
        for v in range(0, height, 32):
            for u in range(0, width, 32):
                z = frame[v, u]
                if np.isfinite(z):
                    pixel = np.array([u, v, 1.0])
                    ray = np.dot(camera_K_inv, pixel)
                    x = ray[0] * z
                    y = ray[1] * z
                    z = ray[2] * z

                    r, g, b = 0, 0, 255
                    rgb = (r << 16) | (g << 8) | b
                    points.append([x, y, z, rgb])

        
        header = Header()
        header.stamp = rospy.Time.now()
        #header.frame_id = "uav1/rgbd/color_optical" 
        header.frame_id = self.camera_frame

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1)
        ]

        cloud_msg = pc2.create_cloud(header, fields, points) 
        self.pub_pointcloud.publish(cloud_msg)
        

if __name__ == '__main__':
    node_instance = MonocularDepthEstimatorNode()
    rospy.spin()
