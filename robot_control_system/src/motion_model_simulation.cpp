#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <eigen3/Eigen/Core>
#include <yaml-cpp/yaml.h>
#include "dwa_nav.h"

Eigen::Matrix<double,1,5> xstate;   ///to store present state of robot  [x, y, yaw, vx, yawrate] for motion computation
Eigen::Matrix<double,1,2> v(2);     ///to store robot speed  [vx, yawrate], also for motion computation
Eigen::Matrix<double,1,5> traj; ////store robot state history
Eigen::Matrix<double,1,3> goal;     ///goal position  [x, y]
nav_msgs::Odometry odom;            ///Odometry infomation to publish
nav_msgs::Path path;                ///path information to draw in rviz

void robotvel_callback(const geometry_msgs::Twist& vel_msg)
{
    ///recevie robot velocity information and store it
    v(0)=vel_msg.linear.x;
    v(1)=vel_msg.angular.z;
    
    ///use present robot state infomation and computed robot velocity to compute next robot state 
    xstate=motion(xstate, v);
    ROS_INFO("x:%f, y:%f, yaw:%f, vx:%f, yawrate:%f",xstate(0),xstate(1),xstate(2),xstate(3),xstate(4));   

    ///because of 2 dimension navigation, only need robot yaw information xstate(2) to compute Quaternion
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(xstate(2));
    
    ///update robot Odometry information
    odom.pose.pose.orientation=odom_quat; 
    odom.pose.pose.position.x=xstate(0);
    odom.pose.pose.position.y=xstate(1);
    odom.twist.twist.linear.x=xstate(3);
    odom.twist.twist.angular.z=xstate(4);
}


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "motion_model_simulation");
    ros::NodeHandle n;

    ///create Subscriber to subscribe robot velocity infomation, and Publisher to publish robot Odometry infomation and robot trajectory infomation
    ros::Subscriber robot_vel_sub = n.subscribe("/cmd_vel", 1000, robotvel_callback);
    ros::Publisher robot_odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 1000);  
    ros::Publisher path_pub = n.advertise<nav_msgs::Path>("trajectory",1, true);
    
    ///create tf broadcaster to brodecast transform from /odom to /base_footprint
    tf::TransformBroadcaster odom_broadcaster;
    
    ///std::string fin = "~/catkin_ws/src/robot_control_system/param/robotconfig.yaml";
    ///YAML::Node yamlConfig = YAML::LoadFile(fin);
    
    ///initialize robot position, goal position from parameter server
    double goalyaw;
    geometry_msgs::Pose goalpos;
    ros::param::get("/goalpoint_x", goalpos.position.x);
    ///goalpos.position.x = yamlConfig["goalpoint_x"].as<float>();
    ros::param::get("/goalpoint_y", goalpos.position.y);
    ros::param::get("/goalpose_yaw", goalyaw);
    ROS_INFO("goalpoint x:%f,y:%f,yaw:%f", goalpos.position.x, goalpos.position.y, goalyaw);
    
    double inityaw;
    geometry_msgs::Pose initpos;
    ros::param::get("/initpoint_x", initpos.position.x);
    ros::param::get("/initpoint_y", initpos.position.y);
    ros::param::get("/initpose_yaw", inityaw);
    ROS_INFO("initpoint x:%f,y:%f,yaw:%f", initpos.position.x, initpos.position.y, inityaw);
    
    
    ////initialize robot state, va and yawrate, goal position, distance to goal=0
    xstate<<initpos.position.x, initpos.position.y, inityaw, 0, 0;   
    goal<<goalpos.position.x, goalpos.position.y, goalyaw;
    v<<0,0;
    double dist_to_goal=0; 
    double yaw_to_goal=0;
    ///use time to store time infomation for trajectory stamp
    ros::Time current_time;
    current_time = ros::Time::now();

    ///use Path to stoe path infomation
    nav_msgs::Path path;
    path.header.stamp=current_time;
    path.header.frame_id="odom";
    
    int goalreach=0;
    
    ros::Rate loop_rate(10);
    while(n.ok()){
        ///listen to robot velocity infomation, to update Odometry information
        ros::spinOnce();    
        
        ///create PoseStamped to store and publish trajectory infomation 
        current_time = ros::Time::now();
        geometry_msgs::PoseStamped this_pose_stamped;
        this_pose_stamped.pose.position.x = xstate(0);
        this_pose_stamped.pose.position.y = xstate(1);
        ///transform yaw to Quaternion
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(xstate(2));
        this_pose_stamped.pose.orientation.x = odom_quat.x;
        this_pose_stamped.pose.orientation.y = odom_quat.y;
        this_pose_stamped.pose.orientation.z = odom_quat.z;
        this_pose_stamped.pose.orientation.w = odom_quat.w;
        ///set time and frame info of current PoseStamped
        this_pose_stamped.header.stamp=current_time;
        this_pose_stamped.header.frame_id="odom";
        path.poses.push_back(this_pose_stamped);
        path_pub.publish(path);

        ///use TransformStamped to broadcaste transform between /odom and /base_footprint
        geometry_msgs::TransformStamped odom_trans;
        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "/odom";
        odom_trans.child_frame_id = "/base_footprint";
        odom_trans.transform.translation.x = odom.pose.pose.position.x;
        odom_trans.transform.translation.y = odom.pose.pose.position.y;
        odom_trans.transform.rotation = odom_quat;
        ///send the transform
        odom_broadcaster.sendTransform(odom_trans);
        
        ///compute current position between goal and robot
        dist_to_goal = sqrt(pow((xstate[0] - goal[0]),2) + pow((xstate[1] - goal[1]),2));
        
        if(goalreach==0 && dist_to_goal <= config.robot_radius*0.1){
            ROS_INFO("Goal position is reached!!!");
            goalreach=1;
            yaw_to_goal = goal[2]-xstate(2);
            while(yaw_to_goal>M_PI) xstate(2)-=2*M_PI;
            while(yaw_to_goal<-M_PI) xstate(2)+=2*M_PI;
            
        }
        if(goalreach==1 && (abs(yaw_to_goal)<=0.2 || abs(abs(yaw_to_goal)-2*M_PI)<0.2 )){
            ROS_INFO("Goal yaw is reached!!!");
            break;
        }
        ///else if goal is not reached, publish Odometry infomation
        odom.header.stamp = current_time;
        odom.header.frame_id = "/odom";
        odom.child_frame_id = "/base_footprint";
        robot_odom_pub.publish(odom);
        loop_rate.sleep();
    }
}


