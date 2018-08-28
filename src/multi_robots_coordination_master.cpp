#include <multi_robots_coordination/multi_robots_coordination_master.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>

namespace multi_robots_coordination{

MultiRobotsCoordinationMaster::MultiRobotsCoordinationMaster(tf::TransformListener& tf, geometry_msgs::PoseStamped p1, geometry_msgs::PoseStamped p2):
tf_(tf),
goal1(p1),
goal2(p2)
{   
    ros::NodeHandle nh;
    send_path1 = nh.serviceClient<multi_robots_coordination::SendPath>("send_path1");
    send_path2 = nh.serviceClient<multi_robots_coordination::SendPath>("send_path2");
    SendPath srv1, srv2;

    astar_planner = new AStarPlanner(tf_);
    
    //获取当前位置
    // odom_sub1 = nh.subscribe<nav_msgs::Odometry>("robot1/odom", 1, boost::bind(&MultiRobotsCoordinationMaster::odomCB1, this, _1));
    // odom_sub2 = nh.subscribe<nav_msgs::Odometry>("robot2/odom", 1, boost::bind(&MultiRobotsCoordinationMaster::odomCB2, this, _1));            
    

    tf::Stamped<tf::Pose> current_pose1;
    tf::Stamped<tf::Pose> current_pose2;
    if(!astar_planner->getRobotPose(current_pose1)){
        ROS_WARN("Unable to get starting pose of robot1, unable to create global plan");
    }
    if(!astar_planner->getRobotPose(current_pose2)){
        ROS_WARN("Unable to get starting pose of robot2, unable to create global plan");
    }//plan_node.cpp/82

    tf::poseStampedTFToMsg(current_pose1, start1);
    tf::poseStampedTFToMsg(current_pose2, start2);
    
    std::vector<geometry_msgs::PoseStamped> plan1;
    std::vector<geometry_msgs::PoseStamped> plan2;

    //规划路径
    if(!astar_planner->makePlan(start1, goal1, plan1)){
        ROS_INFO("Trajectory1 planner failed to plan!\n");
    }
    if(!astar_planner->makePlan(start2, goal2, plan2)){
        ROS_INFO("Trajectory2 planner failed to plan!\n");
    }

    //将plan转为path格式
    for(int i = 0; i < plan1.size(); ++i)   srv1.request.path.poses[i] = plan1[i];
    for(int i = 0; i < plan2.size(); ++i)   srv2.request.path.poses[i] = plan2[i];

    //分别发送
    send_path1.call(srv1);
    send_path2.call(srv2);
}

MultiRobotsCoordinationMaster::~MultiRobotsCoordinationMaster(){
    delete astar_planner;
}

// void MultiRobotsCoordinationMaster::odomCB1(const nav_msgs::Odometry::ConstPtr &odom){
//         //current_pose1 = odom->pose.pose;
//         //printf("%f, %f, %f, %f\n", current_pose.position.x, current_pose.position.y, current_pose.orientation.z, current_pose.orientation.w);
//     }

// void MultiRobotsCoordinationMaster::odomCB2(const nav_msgs::Odometry::ConstPtr &odom){
//         //current_pose2 = odom->pose.pose;
//         //printf("%f, %f, %f, %f\n", current_pose.position.x, current_pose.position.y, current_pose.orientation.z, current_pose.orientation.w);
//     }



};
