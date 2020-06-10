#include <ros/ros.h>
#include <ros/console.h>

#include <std_msgs/String.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <tf/tf.h>
#include <visualization_msgs/Marker.h>

geometry_msgs::Pose robotpose;
void robotposcallback(nav_msgs::Odometry robotpos)
{

    robotpose.position=robotpos.pose.pose.position;
    robotpose.orientation=robotpos.pose.pose.orientation;
    
}

void marker_set_config(visualization_msgs::Marker &marker,int id, double x, double y, double yaw, double r, double g, double b ){
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time();
    marker.ns = "my_namespace";
    marker.id = id;
    marker.pose.position.x = x;
    marker.pose.position.y = y;
    geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(yaw);
    marker.pose.orientation= odom_quat;
    marker.scale.x = 1;
    marker.scale.y = 0.1;
    marker.scale.z = 0.1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
}
int main (int argc, char **argv)
{
    
    ros::init (argc, argv, "drawer");
    ros::NodeHandle n;
    ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("marker",1000);
    ros::Subscriber robotpos_sub = n.subscribe("/odom", 1000, robotposcallback);
    
    /////获取机器人初始位置和姿态,目标位置，最大加速度
    geometry_msgs::Pose initpos;
    double inityaw;
    ros::param::get("/initpoint_x", initpos.position.x);
    ros::param::get("/initpoint_y", initpos.position.y);
    ros::param::get("/initpose_yaw", inityaw);
    double goalyaw;
    geometry_msgs::Pose goalpos;
    ros::param::get("/goalpoint_x", goalpos.position.x);
    ros::param::get("/goalpoint_y", goalpos.position.y);
    ros::param::get("/goalpose_yaw", goalyaw);
    
    visualization_msgs::Marker initpoint;
    initpoint.type = visualization_msgs::Marker::ARROW;
    initpoint.action = visualization_msgs::Marker::ADD;
    marker_set_config(initpoint, 0, initpos.position.x, initpos.position.y, inityaw, 1, 0, 0);
    ////set marker config(marker, id, position x, position y, yaw, red, green, blue)
    
    visualization_msgs::Marker robotpoint;
    robotpoint.type = visualization_msgs::Marker::ARROW;
    robotpoint.action = visualization_msgs::Marker::ADD;
    marker_set_config(robotpoint, 2, initpos.position.x, initpos.position.y, inityaw, 0, 0, 1);
    
    visualization_msgs::Marker goalpoint;
    goalpoint.type = visualization_msgs::Marker::ARROW;
    goalpoint.action = visualization_msgs::Marker::ADD;
    marker_set_config(goalpoint, 1, goalpos.position.x, goalpos.position.y, goalyaw, 0, 1, 0);
    
    ros::Rate loop_rate(100);
    while(ros::ok()){
        ros::spinOnce();
        robotpoint.pose.position=robotpose.position;
        robotpoint.pose.orientation=robotpose.orientation;
        marker_pub.publish( initpoint );
        marker_pub.publish( goalpoint );
        marker_pub.publish( robotpoint);
        loop_rate.sleep();
    }
    return 0;
}
