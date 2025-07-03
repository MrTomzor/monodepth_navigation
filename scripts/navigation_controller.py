#!/usr/bin/env python3

# ROS core
import rospy

# ROS messages
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PointStamped
from mrs_msgs.msg import VelocityReferenceStamped
from cv_bridge import CvBridge

# ROS utilities
import sensor_msgs.point_cloud2 as pc2
import tf2_ros
import tf2_geometry_msgs.tf2_geometry_msgs
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


# Math
import numpy as np




class NavigationControllerNode:
    def __init__(self):
        
        rospy.init_node('navigation_controller', anonymous=True)
        #input_pointclouds_topic_name = '/ov_msckf/points_slam'
        input_pointclouds_topic_name = '/infra/pointcloud'
        #input_pointclouds_topic_name = rospy.get_param("input_topic")
        #input_pointclouds_topic_name = '/midas/pointcloud'

        rospy.Subscriber(input_pointclouds_topic_name, PointCloud2, self.pointcloud_callback)

        self.pointcloud = None
        self.timer = rospy.Timer(rospy.Duration(0.001), self.callback)
     
        rospy.loginfo("Listening on topics: " + input_pointclouds_topic_name)
        rospy.loginfo("Node Started")
        
        self.pub_velocity = rospy.Publisher('/uav1/control_manager/velocity_reference', VelocityReferenceStamped, queue_size=10)

        self.bridge = CvBridge()
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.safe_distance = 3.0



    def callback(self, event):
        if not self.pointcloud:
            #rospy.logwarn("There is no pointcloud to navigate\n")
            return

        left_min_distance, front_min_distance, right_min_distance = self.get_distances_from_sectors(self.pointcloud, 2)
        rospy.loginfo("DISTS L: " + str(left_min_distance) + " FRONT: " + str(front_min_distance) + " R: " + str(right_min_distance))

        
        if front_min_distance <= self.safe_distance:
            if left_min_distance >= self.safe_distance:
                rospy.loginfo(f"Turning left (left_min_distance={left_min_distance:.2f})")
                self.send_velocity_command(vx=0.0, vy=0.0, vz=0.0, yaw_rate=2)
            elif right_min_distance >= self.safe_distance:
                rospy.loginfo(f"Turning right (right_min_distance={right_min_distance:.2f})")
                self.send_velocity_command(vx=0.0, vy=0.0, vz=0.0, yaw_rate=-2)
        else:
            rospy.loginfo(f"Moving forward (front_min_distance={front_min_distance:.2f})")
            self.send_velocity_command(vx=0.5, vy=0.0, vz=0.0, yaw_rate=0.0)

        self.pointcloud = None

        
        
    def pointcloud_callback(self, cloud_msg):
        source_frame = cloud_msg.header.frame_id 
        target_frame = 'uav1/fcu_untilted'
        cloud_time = cloud_msg.header.stamp
        self.pointcloud = self.change_points_frame(source_frame, target_frame, cloud_msg, cloud_time)
        # rospy.loginfo(f"Successfully transformed {len(transformed_points)} points to {target_frame}")


    def get_distances_from_sectors(self, pointcloud, sampling_step):
        left_min_distance = float('inf')
        front_min_distance = float('inf')
        right_min_distance = float('inf')

        for i, point in enumerate(pointcloud):
            # if i % sampling_step != 0:
            #     continue 

            x, y, z = point

            if not (-0.1 < z < 0.1):
                continue
            angle_rad = np.arctan2(y, x)
            angle_deg = np.degrees(angle_rad)

            if angle_deg < -90 or angle_deg > 90:
                continue

            # distance = np.sqrt(x**2 + y**2 + z**2)
            distance = np.sqrt(x**2 + y**2)

            if -90 <= angle_deg < -30:
                right_min_distance = min(right_min_distance, distance)
            elif -30 <= angle_deg <= 30:
                front_min_distance = min(front_min_distance, distance)
            elif 30 < angle_deg <= 90:
                left_min_distance = min(left_min_distance, distance)

        return left_min_distance, front_min_distance, right_min_distance
    


        
    def change_points_frame(self, source_frame, target_frame, cloud_msg, cloud_time):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                source_frame,
                cloud_time,
                rospy.Duration(0.2)
            )
            transformed_cloud_msg = do_transform_cloud(cloud_msg, transform)
            transformed_points = list(pc2.read_points(
                transformed_cloud_msg,
                field_names=("x", "y", "z"),
                skip_nans=True
            ))

            return transformed_points

        except Exception as e:
            rospy.logwarn(f"Failed to transform cloud: {e}")
            return []
        
    
    def send_velocity_command(self, vx, vy, vz, yaw_rate):
        msg = VelocityReferenceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'uav1/fcu_untilted'

        msg.reference.velocity.x = vx
        msg.reference.velocity.y = vy
        msg.reference.velocity.z = vz

        msg.reference.use_heading_rate = True
        msg.reference.heading_rate = yaw_rate

        self.pub_velocity.publish(msg)

        
    

if __name__ == '__main__':
    node_instance = NavigationControllerNode()
    rospy.spin()
