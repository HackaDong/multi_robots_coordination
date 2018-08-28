#include <multi_robots_coordination/multi_robots_coordination_slaver.h>

int main(int argc, char** argv){
    ros::init(argc, argv, "multi_robots_coordination_slaver_node");
    multi_robots_coordination::MultiRobotsCoordinationSlaver MRCS();

    ros::spin();
    
    return 0;
}