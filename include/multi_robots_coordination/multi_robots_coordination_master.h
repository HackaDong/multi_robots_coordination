#ifndef MULTI_ROBOTS_COORDINATION_MASTER_H
#define MULTI_ROBOTS_COORDINATION_MASTER_H

#include <ros/ros.h>
#include <multi_robots_coordination/SendPath.h>
#include <multi_robots_coordination/astar_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

namespace multi_robots_coordination{
class MultiRobotsCoordinationMaster
{
public:
    MultiRobotsCoordinationMaster(tf::TransformListener& tf, geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2);
    virtual ~MultiRobotsCoordinationMaster();
    // void odomCB1(const nav_msgs::Odometry::ConstPtr &odom);
    // void odomCB2(const nav_msgs::Odometry::ConstPtr &odom);
    void sendPath1Service(multi_robots_coordination::SendPath);
    void sendPath2Service(multi_robots_coordination::SendPath);
private:
    tf::TransformListener& tf_;
    AStarPlanner* astar_planner;
    ros::ServiceClient send_path1;
    ros::ServiceClient send_path2;
    // nav_msgs::Path path1;
    // nav_msgs::Path path2;
    geometry_msgs::PoseStamped goal1;
    geometry_msgs::PoseStamped goal2;
    // ros::Subscriber odom_sub1;
    // ros::Subscriber odom_sub2;
    geometry_msgs::PoseStamped start1;
    geometry_msgs::PoseStamped start2;

};
};

#endif