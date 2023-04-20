// OFFBOARD POSITION CONTROL TO MANEUVER THE DRONE 

/** ROS HEADERS **/
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include<std_msgs/Int8.h>
#include<std_msgs/Float64.h>
#include<geometry_msgs/Twist.h>
#include<mavros_msgs/State.h>
#include<mavros_msgs/SetMode.h>
#include<mavros_msgs/PositionTarget.h>
#include<mavros_msgs/CommandBool.h>
#include<mavros_msgs/CommandTOL.h>
#include<trajectory_msgs/MultiDOFJointTrajectory.h>

/** PATH **/
#include<nav_msgs/Path.h>

#include<Eigen/Dense>

#include<iostream>
#include<vector>
#include<math.h>
#include<iterator>
#include<cmath>


trajectory_msgs::MultiDOFJointTrajectory trajectory; //this receives the waypoint every time subscriber is called

Eigen::Vector3d currPose;
int i=0;
int count;
bool TrjaUpdated = 0;
/** waypoint callback **/

void trajectory_cb(const trajectory_msgs::MultiDOFJointTrajectory traj)
{
    trajectory = traj;
    TrjaUpdated = 1;
    i =0;
}
/** drone pose callback **/
void local_pose_cb(const geometry_msgs::PoseStamped pose)
{
    currPose(0) = pose.pose.position.x;
    currPose(1) = pose.pose.position.y;
    currPose(2) = pose.pose.position.z;
}

/** MAVROS OFFBOARD Control **/
void control(ros::Publisher pub, ros::Rate rate, ros::ServiceClient  set_mode_client)
{   
    geometry_msgs::PoseStamped waypoint;
    std::cout<< "here"<<std::endl;
    if((currPose(0)-trajectory.points[i].transforms[0].translation.x) < 0.5 && i< 4)
    {
    waypoint.pose.position.x = trajectory.points[i].transforms[0].translation.x;
    waypoint.pose.position.y = trajectory.points[i].transforms[0].translation.y;
    waypoint.pose.position.z = trajectory.points[i].transforms[0].translation.z;

    waypoint.pose.orientation = trajectory.points[i].transforms[0].rotation;
    
     std::cout<< waypoint <<std::endl; 
    i++;
    }
    std::cout<< "here out"<<std::endl;
    std::cout<< i <<std::endl; 
    //   std::cout<<"Published point "<<"X"<<way_pose.pose.position.x << "y"<< way_pose.pose.position.y<<"z"<< way_pose.pose.position.z<<std::endl;
    // pub.publish(waypoint);
    // mavros_msgs::SetMode offb_set_mode;
    // offb_set_mode.request.custom_mode = "OFFBOARD";

    // mavros_msgs::CommandBool arm_cmd;
    // arm_cmd.request.value = true;

    //     if(set_mode_client.call(offb_set_mode) &&
    //         offb_set_mode.response.mode_sent)
    //     {
    //             ROS_INFO("Offboard enabled");
    //     }
    rate.sleep(); 
    
}



int main(int argc, char** argv)
{
    ros::init(argc, argv, "drone_control");
    ros::NodeHandle n;

    ros::Subscriber path = n.subscribe<trajectory_msgs::MultiDOFJointTrajectory>("/mavros/setpoint_trajectory/local",1, trajectory_cb);
    ros::Subscriber loc = n.subscribe<geometry_msgs::PoseStamped>("/mavros/local_position/pose",10,local_pose_cb);

   ros::Publisher wp_pub = n.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);

    /* Services */
    ros::ServiceClient  arming_client = n.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    ros::ServiceClient  landing_client = n.serviceClient<mavros_msgs::CommandTOL>("mavros/cmd/landing");
    ros::ServiceClient  set_mode_client = n.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");
    
    ros::Rate rate(10);
    
    
    std::cout<<"Enter  waypoint"<<std::endl;

while(!TrjaUpdated || !ros::ok())
    {
        std::cout<<"Waiting for Waypoint"<< ros::ok()<<std::endl;
        
        ros::spinOnce();
        rate.sleep();

        if(!ros::ok())
            {
                break;
            }
    }

   

    if(TrjaUpdated)
    {
        geometry_msgs::PoseStamped initPose;

        if(count==0)
        {
            std::cout<<"Controller started ... "<<std::endl;

            initPose.pose.position.x = 0;
            initPose.pose.position.y = 0;
            initPose.pose.position.z = 0;

            for(int i = 0; i<10 && ros::ok(); i++)
            {
                // wp_pub.publish(initPose);
                ros::spinOnce();
                rate.sleep();
            }

            mavros_msgs::SetMode offb_set_mode;
            offb_set_mode.request.custom_mode = "OFFBOARD";

            mavros_msgs::CommandBool arm_cmd;
            arm_cmd.request.value = true;

        /** set mode to offboard **/
            if(set_mode_client.call(offb_set_mode) &&
                    offb_set_mode.response.mode_sent)
                {
                        ROS_INFO("Offboard enabled");
                }

        }

        count++;
        while(ros::ok())
        {
            control(wp_pub, rate,set_mode_client);   
           
            ros::spinOnce();

            if(!ros::ok())
            break;

        }

    
    }

    ros::spinOnce();
    return 0;
}
