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

# MiDaS depth estimation
from monodepth_navigation.midas_extension import MidasExtension  # type: ignore


class MonocularDepthEstimatorNode:
    def __init__(self):
        
        self.node_initialized = False
        rospy.init_node('monocular_depth_estimator', anonymous=True)

        input_img_topic_name = '/uav1/rgbd/color/image_raw'
        input_pointclouds_topic_name = '/ov_msckf/points_slam'
        input_rgbd_color_cam_info_topic_name = '/uav1/rgbd/color/camera_info'

        rospy.Subscriber(input_img_topic_name, Image, self.callback)
        rospy.Subscriber(input_pointclouds_topic_name, PointCloud2, self.pointcloud_callback)
        rospy.Subscriber(input_rgbd_color_cam_info_topic_name, CameraInfo, self.camera_info_callback)

        rospy.loginfo("Listening on topics: " + input_img_topic_name + " " + input_pointclouds_topic_name + " " + input_rgbd_color_cam_info_topic_name)
        rospy.loginfo("Node Started")

        self.pub_rgb = rospy.Publisher('/midas/rgb_view', Image, queue_size=1)
        self.pub_depth = rospy.Publisher('/midas/depth_view', Image, queue_size=1)
        self.pub_scaled_depth = rospy.Publisher('/midas/scaled_depth_view', Image, queue_size=1)
        self.pub_pointcloud = rospy.Publisher('/midas/pointcloud', PointCloud2, queue_size=1)

        
        self.bridge = CvBridge()
        self.midas = MidasExtension(model_type="MiDaS_small")
        #self.midas = MidasExtension(model_type="DPT_Large")
        #self.midas = MidasExtension(model_type="DPT_Hybrid")

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.camera_info_received = False
        self.camera_K = None 

        self.image = None  
        self.depth_map = None
        self.scaled_depth_map = None

        self.node_initialized = True


    def callback(self, image_msg):
        if not self.node_initialized:
            return

        self.image = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
        
        if self.image is not None:
            depth_color, self.depth_map = self.midas.run(self.image)
            
            # ros_rgb = self.bridge.cv2_to_imgmsg(image, encoding="bgr8")                           # source image
            # self.pub_rgb.publish(ros_rgb)

            self.depth_map = 1.0 / (self.depth_map)
          
            ros_depth = self.bridge.cv2_to_imgmsg(depth_color, encoding="bgr8")                     # MiDas depth map before scaling
            self.pub_depth.publish(ros_depth) 
            
            # scaled_depth_ros = self.bridge.cv2_to_imgmsg(self.scaled_depth_map, encoding="32FC1")   # MiDas depth map after scaling
            # self.pub_scaled_depth.publish(scaled_depth_ros)

            vis_depth = 1.0 / (self.scaled_depth_map) 
            vis_depth = np.nan_to_num(vis_depth, nan=0.0, posinf=0.0, neginf=0.0)

            depth_vis = cv2.normalize(vis_depth, None, 0, 255, cv2.NORM_MINMAX)
            depth_vis = depth_vis.astype(np.uint8)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_MAGMA)

            scaled_depth_vis_ros = self.bridge.cv2_to_imgmsg(depth_vis, encoding="bgr8")
            self.pub_scaled_depth.publish(scaled_depth_vis_ros)

            self.create_pointcloud(self.scaled_depth_map)

            # scaled_depth_ros = self.bridge.cv2_to_imgmsg(self.scaled_depth_map, encoding="32FC1")
            # self.pub_scaled_depth.publish(scaled_depth_ros)

        else:
            rospy.logwarn("Failed to decode image")
    
    def camera_info_callback(self, cam_info_msg):
        if not self.node_initialized:
            return

        if not self.camera_info_received:
            self.camera_K = cam_info_msg.K 
            self.camera_info_received = True 
            self.camera_K = np.array(self.camera_K).reshape(3, 3)
            rospy.loginfo(f"Received camera info: {self.camera_K}")
    
    def pointcloud_callback(self, cloud_msg):
        if not self.node_initialized:
            return

        if not self.camera_info_received:
            rospy.logwarn("Camera intrinsics not yet received.")
            return
        
        pointcloud = list(pc2.read_points(cloud_msg, field_names=("x", "y", "z"), skip_nans=True))

        # Transform points from fish aye to target frame
        source_frame = cloud_msg.header.frame_id 
        target_frame = 'uav1/rgbd/color_optical'
        cloud_time = cloud_msg.header.stamp
        transformed_points = self.change_points_frame(source_frame, target_frame, pointcloud, cloud_time)

        # Projecting 3D points into 2D
        if transformed_points and self.image is not None: 
            pointcloud_2d = self.project_points_3d_to_2d(transformed_points)

            debug_image = self.image.copy()
            pred_depths = []
            real_norms = []
            
            for (u, v, d) in pointcloud_2d:
                u = int(u)
                v = int(v)
                d = int(d)
                # Drawing 2D points
                cv2.circle(debug_image, (u, v), 10, (0, 255, 0), -1)
                if 0 <= u < self.depth_map.shape[1] and 0 <= v < self.depth_map.shape[0]:
                    pred_depth = self.depth_map[v, u]
                    pred_depths.append(pred_depth)
                    real_norms.append(d)

            #scaling depth map
            pred_depths = np.array(pred_depths)
            real_norms = np.array(real_norms)
            scale = np.median(real_norms / pred_depths)
            self.scaled_depth_map = self.depth_map * scale

            rospy.loginfo(f"Received {len(pointcloud)} 3D cloudpoints. \n Transformed {len(transformed_points)} points. \n Projected {len(pointcloud_2d)} points. \n First 3d point is {pointcloud[0]} same poin in 2d {u, v, d} \n Calculated scale is {scale} \n\n\n")
            ros_debug_image = self.bridge.cv2_to_imgmsg(debug_image, encoding="bgr8")           # RGBD color image with 2d pointclouds
            self.pub_rgb.publish(ros_debug_image)
            
        else: 
            rospy.logwarn("No points were transformed or no cam image received")

     
    
    def change_points_frame(self, source_frame, target_frame, pointclouds, cloud_time):
        transformed_points = []
        for pc in pointclouds:
            x, y, z = pc
            p = PointStamped()
            p.header.frame_id = source_frame
            p.header.stamp = cloud_time
            p.point.x = x
            p.point.y = y
            p.point.z = z
            try:
                p_transformed = self.tf_buffer.transform(p, target_frame, timeout=rospy.Duration(0.1))
                transformed_points.append((p_transformed.point.x, p_transformed.point.y, p_transformed.point.z))
            except Exception as e:
                rospy.logwarn(f"Transform failed: {e}")
                continue
        return transformed_points
        
    def project_points_3d_to_2d(self, pointclouds):
        pixel_coords = []

        for pc in pointclouds:
            x, y, z = pc
            if z == 0 or z < 0:
                continue

            K = self.camera_K
            u = ((K[0, 0] * x) / z) + K[0, 2]
            v = ((K[1, 1] * y) / z) + K[1, 2]

            distance = np.sqrt(x**2 + y**2 + z**2)

            pixel_coords.append((u, v, distance))

        return pixel_coords
    
    def create_pointcloud(self, frame):
        height, width = frame.shape
        points = []
        camera_K_inv = np.linalg.inv(self.camera_K)
        for v in range(0, height, 2):
            for u in range(0, width, 2):
                z = frame[v, u]
                if np.isfinite(z):
                    pixel = np.array([u, v, 1.0])
                    ray = np.dot(camera_K_inv, pixel)
                    x = ray[0] * z
                    y = ray[1] * z
                    z = ray[2] * z

                    # r = random.randint(0, 255)
                    # g = random.randint(0, 255)
                    # b = random.randint(0, 255)
                    r = 0
                    g = 0
                    b = 255

                    rgb = (r << 16) | (g << 8) | b

                    points.append([x, y, z, rgb])

        
        header = Header()
        header.stamp = rospy.Time.now()
        header.frame_id = "uav1/rgbd/color_optical" 

        fields = [
            PointField('x', 0, PointField.FLOAT32, 1),
            PointField('y', 4, PointField.FLOAT32, 1),
            PointField('z', 8, PointField.FLOAT32, 1),
            PointField('rgb', 12, PointField.UINT32, 1)
        ]

        cloud_msg = pc2.create_cloud(header, fields, points)                    # Pointcloud from scaled MiDas depth map
        self.pub_pointcloud.publish(cloud_msg)
        

if __name__ == '__main__':
    node_instance = MonocularDepthEstimatorNode()
    rospy.spin()
