#include <multi_robots_coordination/astar_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <vector>

int main(int argc, char** argv){
    ros::init(argc, argv, "test");
    tf::TransformListener tf(ros::Duration(10));
    geometry_msgs::PoseStamped start, end;
    start.header.frame_id = "/map";
    end.header.frame_id = "/map";
    start.pose.position.x = 1.0;
    start.pose.position.y = -1.0;
    end.pose.position.x = 0;
    end.pose.position.y = 3.5;
    std::vector<geometry_msgs::PoseStamped> plan;

    multi_robots_coordination::AStarPlanner ASP(tf);
    ASP.makePlan(start, end, plan);

    printf("plan size = %d\n", plan.size());

    for(int i = 0; i < plan.size(); ++i){
        printf("point %d, x = %f, y = %f\n", i, plan[i].pose.position.x, plan[i].pose.position.y);
    }

    ros::spin();
    
    return 0;
}