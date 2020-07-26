#ifndef DWA_NAV_H
#define DWA_NAV_H

#include <eigen3/Eigen/Core>
using namespace Eigen;

class Config{
    public:
        double max_speed = 2.0;                     /// [m/s]
        double min_speed = 0;                       /// [m/s]
        double max_yawrate = 120.0 * M_PI / 180.0;   /// [rad/s]               
        double max_dyawrate = 90.0 * M_PI / 180.0;  /// [rad/ss]
        double v_reso = 0.01;                       /// [m/s]
        double yawrate_reso = 0.1 * M_PI / 180.0;   /// [rad/s]
        double dt = 0.05;                    /// [s] Time tick for motion prediction
        double predict_time = 2.0;          /// [s]
        double to_goal_cost_gain = 0.15;
        double speed_cost_gain = 1.0;
        double obstacle_cost_gain = 1.0;
        double robot_radius = 0.05;          ///[m] for collision check
};

class motion_config{
    public:
        double max_accel=0.0;       /// [m/ss]
};

struct Trajectory{
    Matrix<double,1,5> xs;
    Trajectory* next=NULL;
};

struct uandtraj{
    Matrix<double,1,2> u;
    Trajectory* traj;
};

struct Obstacle{
    Matrix<double,1,2> ob;  ///obstacle [x,y,
    Obstacle* next=NULL;
};

Matrix<double,1,4> calc_dynamic_window(Matrix<double,1,5> xtemp,motion_config moconfig);

Matrix<double,1,5> motion(Matrix<double,1,5> xtemp,Matrix<double,1,2> u);
Trajectory* predict_trajectory(Matrix<double, 1, 5> xstate, double vx, double omega);

double calc_to_goal_cost(Matrix<double, Eigen::Dynamic, 5> traj,Matrix<double,1,3> goal);
double calc_to_goalyaw_cost(Matrix<double, Eigen::Dynamic, 5> traj,Matrix<double,1,3> goal);
double calc_obstacle_cost(Trajectory* trajhead, double obhead[10][2]);

uandtraj calc_final_input(Matrix<double, 1, 5> xstate, Matrix<double,1,2> u,Matrix<double,1,4> dw,Matrix<double,1,3> goal, double obhead[10][2]);
uandtraj calc_final_input_yaw(Matrix<double, 1, 5> xstate, Matrix<double,1,2> u,Matrix<double,1,4> dw,Matrix<double,1,3> goal);

uandtraj dwa_control_dist(Matrix<double, 1, 5> x, Matrix<double,1,2> u, Matrix<double,1,3> goal, motion_config moconfig, double obhead[10][2]);
uandtraj dwa_control_yaw(Matrix<double, 1, 5> x, Matrix<double,1,2> u, Matrix<double,1,3> goal,motion_config moconfig);

#endif 
