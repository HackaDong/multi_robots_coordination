#include <multi_robots_coordination/multi_robots_coordination_master.h>


int main(int argc, char** argv){
    ros::init(argc, argv, "multi_robots_coordination_master_node");
    tf::TransformListener tf(ros::Duration(10));

    geometry_msgs::PoseStamped goal1;
    geometry_msgs::PoseStamped goal2;
    multi_robots_coordination::MultiRobotsCoordinationMaster MRCM(tf, goal1, goal2);

    ros::spin();
    
    return 0;
}