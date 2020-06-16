#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <eigen3/Eigen/Core>
#include <dwa_nav.h>

Eigen::Matrix<double,1,5> xstate;  ///to store present state of robot  [x, y, yaw, vx, yawrate]
Eigen::Matrix<double,1,2> v(2);    ///to store robot speed  [vx, yawrate], also for dwa_control computation
Eigen::Matrix<double,1,3> goal;    ///goal position  [x, y]
int Task_state=0;  
  
void robotpose_callback(const nav_msgs::Odometry& odommsg)
{
    ////transform Quaternion information from odometry to eular angle, because orientation in odometry is Quaternion and we need yaw to compute
    tf::Quaternion quat;
    tf::quaternionMsgToTF(odommsg.pose.pose.orientation, quat);
    double roll, pitch, yaw; 
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    
    ////use odometry information to update robot state
    xstate(0)=odommsg.pose.pose.position.x;
    xstate(1)=odommsg.pose.pose.position.y;
    xstate(2)=yaw; 
    xstate(3)=odommsg.twist.twist.linear.x;
    xstate(4)=odommsg.twist.twist.angular.z; 
}

///for change task state from distance navigation to yaw navigation after robot reach goal position but yaw is not right
void taskstate_callback(std_msgs::Int8 task_state){
    if(task_state.data==1) Task_state=1;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "controller");
    ros::NodeHandle n;

    ///create publisher to publish robot velocity information, and Subscriber to subscribe odometry information 
    ros::Publisher robot_vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
    ros::Subscriber robot_odom_sub = n.subscribe("/odom", 1000, robotpose_callback);
    ros::Subscriber task_state_sub = n.subscribe("/task_state",1,taskstate_callback);
    ///initialize robot position, goal position and maximum acceleration from parameter server
    double inityaw;
    geometry_msgs::Pose initpos;
    ros::param::get("/initpoint_x", initpos.position.x);
    ros::param::get("/initpoint_y", initpos.position.y);
    ros::param::get("/initpose_yaw", inityaw);
    double goalyaw;
    geometry_msgs::Pose goalpos;
    ros::param::get("/goalpoint_x", goalpos.position.x);
    ros::param::get("/goalpoint_y", goalpos.position.y);
    ros::param::get("/goalpose_yaw", goalyaw);
    float maxacc;
    ros::param::get("/maxacceleration", maxacc);
    motion_config moconfig;
    moconfig.max_accel=maxacc;
    
    ///use Twist vxandomega to store and publish vx and yawrate, initialize with 0, 0
    geometry_msgs::Twist vxandomega;
    vxandomega.linear.x=0;
    vxandomega.angular.z=0;
    
    ///initialize robot state, va and yawrate, goal position
    xstate<< initpos.position.x, initpos.position.y, inityaw, 0, 0;   
    goal<<goalpos.position.x, goalpos.position.y, goalyaw;
    v<<0, 0;  
        
    ///create struct to store [vx, yawrate] and trajectory of robot
    uandtraj results;
    results.traj=NULL;
    
    ///initialize Obstacle
    Obstacle* obhead=new Obstacle;
    obhead->ob<<4.030657,-0.5136;
    Obstacle* obtemp=new Obstacle;
    obtemp->ob<<4,-1;
    obhead->next=obtemp;
    
    
    ros::Rate loop_rate(10);
    while (ros::ok())
    {
        
        ///use present robot state and velocity information to compute next velocity information
        ///double dist_to_goal;
        ///if(results.traj==NULL) dist_to_goal=10;
        ///else dist_to_goal=sqrt(pow(results.traj->xs(0)-goal(0),2)+pow(results.traj->xs(1)-goal(1),2));

        if(Task_state==0) results=dwa_control_dist(xstate,v,goal,moconfig, obhead);
        else if(Task_state==1) results=dwa_control_yaw(xstate,v,goal,moconfig);
        
        v<<results.u(0),results.u(1);
        
        ///publish robot velocity information
        vxandomega.linear.x=results.u(0);
        vxandomega.angular.z=results.u(1);
        robot_vel_pub.publish(vxandomega);
        
        ///listen to odometry information
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    delete obhead;
    return 0;
}

