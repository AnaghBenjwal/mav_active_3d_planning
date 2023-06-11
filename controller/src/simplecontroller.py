#!/usr/bin/env python

import time
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
import rospy
from sensor_msgs.msg import Image, PointCloud2
from geometry_msgs.msg import PoseStamped, Vector3, Quaternion
from trajectory_msgs.msg import MultiDOFJointTrajectory
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, CommandBoolRequest, SetMode, SetModeRequest


# local_pos_pub = None
# local_vel_pub = None
# local_acc_pub = None

def setArm():
    # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
    rospy.wait_for_service('/mavros/cmd/arming')  # Waiting untill the service starts 
    try:
        armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
        armService(True)
    except rospy.ServiceException as e:
        print ("Service arming call failed: %s"%e)

# def DisArm():
#     # Calling to /mavros/cmd/arming to arm the drone and print fail message on failure
#     rospy.wait_for_service('/mavros/cmd/arming')  # Waiting untill the service starts 
#     try:
#         armService = rospy.ServiceProxy('/mavros/cmd/arming', CommandBool) # Creating a proxy service for the rosservice named /mavros/cmd/arming for arming the drone 
#         armService(False)
#     except rospy.ServiceException as e:
#         print ("Service disarming call failed: %s"%e)

def offboard_set_mode():
    rospy.wait_for_service('/mavros/set_mode')
    try: 
        SetModeOffboard = rospy.ServiceProxy('/mavros/set_mode',SetMode) 
        SetModeOffboard(custom_mode='OFFBOARD')
    except rospy.ServiceException as e:
        print ("Service calling offboard failed: %s"%e)

# def land_set_mode():
#     rospy.wait_for_service('/mavros/set_mode')
#     try: 
#         SetMode = rospy.ServiceProxy('/mavros/set_mode', SetMode) 
#         SetMode(custom_mode='AUTO.LAND')
#     except rospy.ServiceException as e:
#         print ("Service calling Land failed: %s"%e)


###############################  callback functions  ###########################################
current_state = State()
def state_callback(msg):
    # Callback function for topic /mavros/state
    global current_state
    current_state = msg

local_pose = PoseStamped()
def pose_callback(msg):
    # Extract the relevant information from the received message
    global local_pose
    local_pose = msg.pose  # Get the pose from the PoseStamped message
    # position = pose.position
    # orientation = pose.orientation

def depth_image_callback(msg):
    # Extract the relevant information from the received message
    # depth_image = CvBridge().imgmsg_to_cv2(msg)
    cv_depth_image = CvBridge().imgmsg_to_cv2(msg, '16UC1')
    

def image_callback(msg):
    # Extract the relevant information from the received message
    cv_rgb_image = CvBridge().imgmsg_to_cv2(msg,desired_encoding='bgr8')

planner_setpoint_pose = Vector3()
planner_setpoint_orientation = Quaternion()
def trajectory_callback(msg):
    global planner_setpoint_pose
    global planner_setpoint_orientation
    # Extract the relevant information from the received message
    transforms = msg.points[0].transforms
    planner_setpoint_pose = transforms[0].translation
    planner_setpoint_orientation = transforms[0].rotation
  
  
def reached_setpoint(setpoint_pose) :
    global local_pose
    # global setpoint_pose
    
    desired = np.array((setpoint_pose.x, setpoint_pose.y, setpoint_pose.z))
    current = np.array((local_pose.position.x, local_pose.position.y, local_pose.position.z))

    return (np.linalg.norm(desired - current) < 1)
  
  
def main() :
    
    global current_state, planner_setpoint_orientation, planner_setpoint_pose 
    
    # rate = rospy.Rate(20)
    rospy.Subscriber("mavros/state", State, state_callback)
    rospy.Subscriber("mavros/local_position/pose", PoseStamped, pose_callback)
    rospy.Subscriber("/mavros/setpoint_trajectory/local", MultiDOFJointTrajectory, trajectory_callback)
    # rospy.Subscriber("camera/rgb/image_raw", Image, image_callback)
    # rospy.Subscriber("camera/depth/image_raw", Image, depth_image_callback)
    # rospy.Subscriber('/camera/depth/points', PointCloud2, pointcloud_callback)
    
    
    local_pos_pub = rospy.Publisher('mavros/setpoint_position/local', PoseStamped, queue_size=10)
    # local_vel_pub= rospy.Publisher('mavros/setpoint_velocity/cmd_vel', Twist, queue_size=10)
    # local_acc_pub = rospy.Publisher('mavros/setpoint_accel/cmd_acc', Accel, queue_size=10)
    
    pose_setpoint = PoseStamped()
    pose_setpoint.pose.position.x = 0
    pose_setpoint.pose.position.y = 0
    pose_setpoint.pose.position.z = 3
    # pose_setpoint.pose.orientation = 
    # pose_setpoint.pose.orientation.x = 0
    # pose_setpoint.pose.orientation.y = 0
    # pose_setpoint.pose.orientation.z = 0
    # pose_setpoint.pose.orientation.w = 0

    
    while not current_state.armed :
        setArm()
        if (current_state.armed == True) : break
    print("Armed")
    time.sleep(5)

    while not current_state.mode=="OFFBOARD":
        # print("inside while")
        for i in range(100):
            local_pos_pub.publish(pose_setpoint)
        offboard_set_mode()
    print ("OFFBOARD mode activated")
    time.sleep(5)
    
    # if (current_state.mode == 'OFFBOARD') : print("Offboard Mode!")
    # print('Publishing First Setpoint!')
    # local_pos_pub.publish(pose_setpoint)
    time.sleep(5)
    
    plannercount = 0
    while (not rospy.is_shutdown()) :
        # pose_setpoint = planner_setpoint_pose
        pose_setpoint.pose.position.x = planner_setpoint_pose.x
        pose_setpoint.pose.position.y = planner_setpoint_pose.y
        pose_setpoint.pose.position.z = planner_setpoint_pose.z
        pose_setpoint.pose.orientation = planner_setpoint_orientation
        
        # pose_setpoint.pose.orientation.x = planner_setpoint_orientation.x
        # pose_setpoint.pose.orientation.y = planner_setpoint_orientation.y
        # pose_setpoint.pose.orientation.z = planner_setpoint_orientation.z
        # pose_setpoint.pose.orientation.w = planner_setpoint_orientation.w
        
        print("Position:",(pose_setpoint.pose.position.x,pose_setpoint.pose.position.y,pose_setpoint.pose.position.z),
              "\nOrientation:",(pose_setpoint.pose.orientation.x,pose_setpoint.pose.orientation.y,pose_setpoint.pose.orientation.z,pose_setpoint.pose.orientation.w))
        
        if (plannercount == 0) : print('Starting Planner Setpoint Navigation')
        # if (plannercount > 1) : print('Obtained new planner setpoint')
        plannercount+=1
        time.sleep(5)
        while (reached_setpoint(pose_setpoint.pose.position) == False) :
            local_pos_pub.publish(pose_setpoint)
            time.sleep(10)
    

if __name__ == '__main__':
    rospy.init_node('simple_controller_node', anonymous=True)
    # print('Inside Main Function')
    main()
    rospy.spin()