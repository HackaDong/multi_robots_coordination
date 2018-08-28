#ifndef MULTI_ROBOTS_COORDINATION_CONTROLLER_H
#define MULTI_ROBOTS_COORDINATION_CONTROLLER_H

#include <multi_robots_coordination/trajectory.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>

namespace multi_robots_coordination{
class Controller{
    public:
        Controller(Trajectory* traj, double controller_frequency, double controller_patience_pose, double controller_patience_theta, double Kx, double Ky, double Ktheta);
        ~Controller();
        //设置当前姿态或者参考姿态
        bool setCurrentPose(geometry_msgs::Pose current_pose);
        bool setCurrentReference();
        bool setNextReference();//设置下个参考姿态
        double getDelta_x_n();
        double getDelta_y_n();
        double getDelta_theta_n();
        double computeTheta_ez_n();
        double computeV_c_n();//计算线速度
        double computeW_c_n();//计算角速度
        bool computePositionControlActions(geometry_msgs::Pose current_pose, geometry_msgs::Twist& twist, int t_i, int p_i);//位置控制循环
        bool computeOrientationControlActions(geometry_msgs::Twist& twist, double angle_diff);//角度控制
    private:
        Trajectory* traj_;
        double controller_frequency_;//控制频率
        double controller_patience_pose_, controller_patience_theta_;//误差容忍度
        double Kx_, Ky_, Ktheta_;//控制参数
        
        double x_n, y_n, theta_n;//当前姿态
        double x_ref_n, y_ref_n, theta_ref_n;//参考姿态
        double x_ref_n_plus_1, y_ref_n_plus_1;//下个时间点的参考姿态
        double w_c_n, v_c_n;//角速度和线速度计算值
        int traj_index, point_index;//路径标识，姿态标识
};
};

#endif