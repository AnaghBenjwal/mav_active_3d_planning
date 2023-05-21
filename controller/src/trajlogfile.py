#!/usr/bin/env python

import rospy
import cv2
import csv
import pcl
import numpy as np
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, Transform
from cv_bridge import CvBridge
import ros_numpy
from trajectory_msgs.msg import MultiDOFJointTrajectory

filepath = '/home/anagh/catkin_ws/src/mav_active_3d_planning/controller/src/logs_140523_1/'

def pointcloud_callback(msg):
    global pcl_counter, pcl_received, img_counter, depth_counter, pose_received, pose_counter, traj_counter

    pcl_counter += 1
    pcl_received = True

    

    # Save point cloud message
    pc_np = ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg, remove_nans=True)
    pc_pcl = pcl.PointCloud(np.array(pc_np, dtype=np.float32))
    filename = filepath + 'camera_depth_points/' + str(pcl_counter) + '_' + str(rospy.get_rostime()) + ".ply"
    pcl.save(pc_pcl, filename)

    # Save image message
    cv_image = CvBridge().imgmsg_to_cv2(image_msg, desired_encoding='bgr8')
    img_filename = filepath + 'camera_rgb_image_raw/' + str(pcl_counter) + '_' + str(rospy.get_rostime()) + ".jpg"
    cv2.imwrite(img_filename, cv_image)
    img_counter += 1

    # Save depth image message
    depth_image = CvBridge().imgmsg_to_cv2(depth_image_msg)
    depth_filename = filepath + 'camera_depth_image_raw/' + str(pcl_counter) + '_' + str(rospy.get_rostime()) + ".jpg"
    cv2.imwrite(depth_filename, depth_image)
    depth_counter += 1

    # Save pose message
    pose = pose_msg.pose
    position = pose.position
    orientation = pose.orientation

    position_data = [position.x, position.y, position.z]
    orientation_data = [orientation.x, orientation.y, orientation.z, orientation.w]
    data = [position_data, orientation_data]

    with open(filepath + 'output_pose.csv', 'a') as posefile:
        writer = csv.writer(posefile)
        writer.writerow(['/mavros/local_position/pose__'+str(pose_counter), rospy.get_time(), position_data, orientation_data])
    
    pose_counter += 1 
    pose_received = True


def image_callback(msg):
    global image_msg

    if pcl_received:
        image_msg = msg


def depth_image_callback(msg):
    global depth_image_msg

    if pcl_received:
        depth_image_msg = msg


def pose_callback(msg):
    global pose_msg

    if pcl_received:
        pose_msg = msg


def trajectory_callback(msg):
    global traj_counter
    # Extract the relevant information from the received message and save it to a CSV file
    transforms = msg.points[0].transforms
    pose = transforms[0].translation
    orientation = transforms[0].rotation
    # Open the CSV file in append mode
    with open(filepath+'output_traj.csv', 'a') as csvfile:
        writer = csv.writer(csvfile)
        # Write the extracted data to the CSV file
        writer.writerow(['/mavros/setpoint_trajectory/local__'+str(traj_counter), ';', rospy.get_time(), ';', pose, ';', orientation])  
    traj_counter += 1


if __name__ == '__main__':
    rospy.init_node('csv_writer_node', anonymous=True)

    # Initialize flags and counters
    pcl_counter = 0
    pcl_received = False
    img_counter = 0
    depth_counter = 0
    pose_counter = 0
    traj_counter = 0
    pose_received = False

    # Subscribe to the desired rostopics
    rospy.Subscriber('/camera/depth/points', PointCloud2, pointcloud_callback)
    rospy.Subscriber('/camera/rgb/image_raw', Image, image_callback)
    rospy.Subscriber('/camera/depth/image_raw', Image, depth_image_callback)
    rospy.Subscriber('/mavros/local_position/pose', PoseStamped, pose_callback)
    rospy.Subscriber('/mavros/setpoint_trajectory/local', MultiDOFJointTrajectory, trajectory_callback)
    rospy.spin()
