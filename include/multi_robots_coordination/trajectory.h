#ifndef MULTI_ROBOTS_COORDINATION_TRAJECTORY_H
#define MULTI_ROBOTS_COORDINATION_TRAJECTORY_H

#include <ros/ros.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Vector3.h>
#include <vector>
#include <tf/tf.h>

namespace multi_robots_coordination{
class Trajectory{
    public:
        Trajectory();
        ~Trajectory();
        //规划单条或多条路径，在class PatrolRobot中调用
        bool makePlan(geometry_msgs::Pose current_pose, std::vector<geometry_msgs::Pose> g, std::vector< std::vector<geometry_msgs::Pose> >& traj, double s);
        //获取到第index1个目标点的路径上的第index2个点的姿态信息
        bool getPositionAt(int index1, int index2, geometry_msgs::Pose& pose);//index1：第几条路径，index2:第几个点
        //计算两点间距离
        double calDistance(double x, double y);
        //计算目标点相对于起点的方位角，用于设置机器人姿态
        geometry_msgs::Quaternion getOrientation(geometry_msgs::Pose p1, geometry_msgs::Pose p2);
        //创建单条路径的均匀路径点
        std::vector<geometry_msgs::Pose> createTrajectory(geometry_msgs::Pose begin, geometry_msgs::Pose end, geometry_msgs::Pose end_n);
        geometry_msgs::Vector3 quaternionToEuler(geometry_msgs::Quaternion q);
        geometry_msgs::Quaternion eulerToQuaterion(geometry_msgs::Vector3 v);
        //获取第index条路径的步数，即点的个数
        int getSteps(int index);
        bool goalReached(int traj_index, geometry_msgs::Pose current_pose, double pos_tolerance);
        bool OrientationReached(int traj_index, geometry_msgs::Pose current_pose, double pos_tolerance, double &angle_diff);
        double angleDiff(double a, double b);//求两个姿态角的差值
        double normalize(double z);
    private:
        std::vector< std::vector<geometry_msgs::Pose> > trajectory;
        std::vector<geometry_msgs::Pose> goals;
        double step_dis;
};
};

#endif