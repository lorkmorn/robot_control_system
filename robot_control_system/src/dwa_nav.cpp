#include <eigen3/Eigen/Core>
#include <math.h>
#include <ros/ros.h>
#include <dwa_nav.h>
#define max(a,b) (((a) > (b)) ? (a) : (b))
#define min(a,b) (((a) < (b)) ? (a) : (b))
using namespace Eigen;

Matrix<double,1,5> motion(Matrix<double,1,5> xtemp,Matrix<double,1,2> u){
    Config config;
    double dt=config.dt;
    Matrix<double,1,5> x=xtemp;
    x[2] += u[1] * dt;
    x[0] += u[0] * cos(x[2]) * dt;
    x[1] += u[0] * sin(x[2]) * dt;
    x[3] = u[0];
    x[4] = u[1];
    return x;
}

Matrix<double,1,4> calc_dynamic_window(Matrix<double,1,5> xtemp,motion_config moconfig){
       Config config;
       Matrix<double,1,5> x=xtemp;
       Matrix<double,1,4> Vs;
       Vs <<config.min_speed, config.max_speed, -config.max_yawrate, config.max_yawrate;
       
       Matrix<double,1,4> Vd; 
       Vd <<x[3]-moconfig.max_accel * config.dt,   x[3] + moconfig.max_accel * config.dt,\
            x[4]-config.max_dyawrate * config.dt,   x[4] + config.max_dyawrate * config.dt;
    
       Matrix<double,1,4> dw;
       dw <<max(Vs[0], Vd[0]), min(Vs[1], Vd[1]), max(Vs[2], Vd[2]), min(Vs[3], Vd[3]);
       return dw;
}


double calc_to_goal_cost(Matrix<double, Eigen::Dynamic, 5> traj,Matrix<double,1,3> goal){
    double dx = goal[0]-traj(0);
    double dy = goal[1]-traj(1);
    double error_dist = sqrt(pow(dx,2)+pow(dy,2));
    double error_angle = atan2(dy, dx);
    double cost=abs(error_angle - traj(2))+error_dist;
    return cost;
}

double calc_to_goalyaw_cost(Matrix<double, Eigen::Dynamic, 5> traj,Matrix<double,1,3> goal){
    double dth = goal[2]-traj(2);
    double cost = abs(dth);
    return cost;
}

Trajectory* predict_trajectory(Matrix<double, 1, 5> xstate, double vx, double omega){
    Config config;
    Matrix<double, 1, 5> x = xstate;
    Trajectory* head=new Trajectory;
    head->xs=x;
    head->next=NULL;
    Trajectory* temp=head;
    Matrix<double,1,2> u;
    u<<vx,omega;
    double time = 0;
    while(time <= config.predict_time){
        x = motion(x, u);
        Trajectory* temptraj=new Trajectory;
        temptraj->xs=x;
        temptraj->next=NULL;
        temp->next=temptraj;
        temp=temp->next;
        time += config.dt;
    }
    return head;
}

double calc_obstacle_cost(Trajectory* trajhead, Obstacle* obhead){
    Config config;
    int skip_n = 2; ///for speed up
    double minr = 9999;
    Trajectory* tempi=trajhead;
    Obstacle* tempj=obhead;
    for(;(*tempi).next!=NULL;tempi=(*tempi).next){
        for(;tempj!=NULL && (*tempj).next!=NULL;tempj=(*tempj).next){
            double ox = (*tempj).ob[0];
            double oy = (*tempj).ob[1];
            double dx = (*tempi).xs[0] - ox;
            double dy = (*tempi).xs[1] - oy;
            double r = sqrt(pow(dx,2) + pow(dy,2));
            if(r <= config.robot_radius*2) return 9999;  //// collision
            if(minr >= r)minr = r;
        }
        for(int i=0;i<skip_n-1;i++) tempi=(*tempi).next;
    }
    return 1.0/minr;
}
    

uandtraj calc_final_input(Matrix<double, 1, 5> xstate, Matrix<double,1,2> u,Matrix<double,1,4> dw,Matrix<double,1,3> goal, Obstacle* obhead){
    Config config;
    Matrix<double, 1, 5> x_init = xstate;
    float min_cost = 9999999;
    Matrix<double,1,2> best_u;
    best_u<<0.0, 0.0;
    Trajectory* best_traj;
    ////best_traj<<x_init;
    for(double v=dw[0];v<dw[1];v+=config.v_reso){
        for(double y=dw[2];y<dw[3];y+=config.yawrate_reso){
            Trajectory* trajhead = predict_trajectory(x_init, v, y);////运动后期CPU速度跟不上，有点卡顿
            Trajectory* trajlast=trajhead;
            while((*trajlast).next!=NULL){trajlast=(*trajlast).next;}
            ///double to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost(traj, goal);
            double to_goal_cost = config.to_goal_cost_gain * calc_to_goal_cost((*trajlast).xs, goal);
            ///double speed_cost = config.speed_cost_gain * (config.max_speed - traj(traj.rows()-1, 3));
            double speed_cost = config.speed_cost_gain * (config.max_speed - (*trajlast).xs[3]);
            double ob_cost = config.obstacle_cost_gain * calc_obstacle_cost(trajhead, obhead);
            ///double ob_cost=0;
            double final_cost = to_goal_cost + speed_cost ;

            //// search minimum trajectory
            if(min_cost >= final_cost){
                min_cost = final_cost;
                ///delete best_traj;
                ///ROS_INFO("to_goal_cost:%f, speed_cost:%f, ob_cost:%f",to_goal_cost,speed_cost,ob_cost);
                best_u<<v, y;
                best_traj=trajhead;
            }
            
            trajlast=trajhead;
            while((*trajlast).next!=NULL){
                trajlast=(*trajlast).next;
                delete trajhead;
                trajhead=trajlast;
            }
        }
        
    }
    uandtraj results;
    results.u=best_u;
    results.traj=best_traj;
    return results;
}

uandtraj calc_final_input_yaw(Matrix<double, 1, 5> xstate, Matrix<double,1,2> u,Matrix<double,1,4> dw,Matrix<double,1,3> goal){
    Config config;
    Matrix<double, 1, 5> x_init = xstate;
    float min_cost = 9999999;
    Matrix<double,1,2> best_u;
    best_u<<0.0, 0.0;
    Trajectory* best_traj;
    ////best_traj<<x_init;
    double v=0;
    for(double y=dw[2];y<dw[3];y+=config.yawrate_reso){
        Trajectory* trajhead = predict_trajectory(x_init, v, y);////运动后期CPU速度跟不上，有点卡顿
        Trajectory* trajlast=trajhead;
        while((*trajlast).next!=NULL){trajlast=(*trajlast).next;}
        double final_cost  = calc_to_goalyaw_cost((*trajlast).xs, goal);
        ///ROS_INFO("trajyaw:%f, goalyaw:%f",(*trajlast).xs(2),goal(2));
        //// search minimum trajectory
        if(min_cost >= final_cost){
            min_cost = final_cost;
            ///delete best_traj;
            best_u<<v, y;
            best_traj=trajhead;
        }
        
        
    }
    uandtraj results;
    results.u=best_u;
    results.traj=best_traj;
    return results;
}

uandtraj dwa_control_dist(Matrix<double, 1, 5> x, Matrix<double,1,2> u, Matrix<double,1,3> goal, motion_config moconfig, Obstacle* obhead){
    Matrix<double,1,4> dw = calc_dynamic_window(x,moconfig);
    uandtraj results= calc_final_input(x, u, dw, goal, obhead);
    return results;
    
}

uandtraj dwa_control_yaw(Matrix<double, 1, 5> x, Matrix<double,1,2> u, Matrix<double,1,3> goal,motion_config moconfig){
    Matrix<double,1,4> dw = calc_dynamic_window(x,moconfig);
    uandtraj results= calc_final_input_yaw(x, u, dw, goal);
    return results;
}

