#!/usr/bin/env python

import rospy
import ros_numpy
from cv_bridge import CvBridge
import cv2
import pcl
import pcl_ros
import numpy as np
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import Image
# from sensor_msgs.msg import CameraInfo
from geometry_msgs.msg import PoseStamped
# from mavros_msgs.msg import Trajectory
from trajectory_msgs.msg import MultiDOFJointTrajectory
# from mavros_msgs.msg import CameraImageCaptured
import csv

filepath = '/home/anagh/catkin_ws/src/mav_active_3d_planning/controller/src/logs_140523_1/'



def pointcloud_callback(msg):
    pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
    pc_pcl = pcl.PointCloud(np.array(pc_np, dtype=np.float32))
    filename = filepath + 'camera_depth_points/' + str(rospy.get_rostime()) + ".ply"
    pcl.save(pc_pcl, filename)



def image_callback(msg):
    # Extract the relevant information from the received message and save it to a CSV file
    
    cv_image = CvBridge().imgmsg_to_cv2(msg,desired_encoding='bgr8')
    filename = filepath + 'camera_rgb_image_raw/' + str(rospy.get_rostime()) + ".jpg"
    cv2.imwrite(filename, cv_image)



def depth_image_callback(msg):
    # Extract the relevant information from the received message and save it to a CSV file
    depth_image = CvBridge().imgmsg_to_cv2(msg)
    cv_image = CvBridge().imgmsg_to_cv2(msg, '16UC1')
    filename = filepath + 'camera_depth_image_raw/' + str(rospy.get_rostime()) + ".jpg"
    cv2.imwrite(filename, cv_image)



def pose_callback(msg):
    # Extract the relevant information from the received message and save it to a CSV file
    pose = msg.pose  # Get the pose from the PoseStamped message
    position = pose.position
    orientation = pose.orientation

    position_data = [position.x, position.y, position.z]
    orientation_data = [orientation.x, orientation.y, orientation.z, orientation.w]
    data = [position_data, orientation_data]
    # Open the CSV file in append mode
    with open(filepath+'output_pose.csv', 'a') as posefile:
        writer = csv.writer(posefile)
        # Write the extracted data to the CSV file
        writer.writerow(['/mavros/local_position/pose', rospy.get_time(), position_data, orientation_data])


        
def trajectory_callback(msg):
    # Extract the relevant information from the received message and save it to a CSV file
    transforms = msg.points[0].transforms
    pose = transforms[0].translation
    orientation = transforms[0].rotation
    # Open the CSV file in append mode
    with open(filepath+'output_traj.csv', 'a') as csvfile:
        writer = csv.writer(csvfile)
        # Write the extracted data to the CSV file
        writer.writerow(['/mavros/setpoint_trajectory/local', ';', rospy.get_time(), ';', pose, ';', orientation])  
    


if __name__ == '__main__':
    rospy.init_node('csv_writer_node', anonymous=True)

    # Subscribe to the desired rostopics
    rospy.Subscriber('/camera/depth/points', PointCloud2, pointcloud_callback)
    rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
    rospy.Subscriber('/camera/depth/image_raw', Image, depth_image_callback)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    rospy.Subscriber('/mavros/setpoint_trajectory/local', MultiDOFJointTrajectory, trajectory_callback)
    
    rospy.spin()