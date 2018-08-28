#ifndef ASTAR_PLANNER_H
#define ASTAR_PLANNER_H

#include <vector>
#include <algorithm>
#include <multi_robots_coordination/static_map.h>
#include <geometry_msgs/PoseStamped.h>
#include <ros/ros.h>
#include <tf/tf.h>
#include <tf/transform_listener.h>

#define POT_HIGH 255
namespace multi_robots_coordination
{

class Index {
    public:
        Index(int a, float b) {
            i = a;
            cost = b;
        }
        int i;
        float cost;
};

struct greater1 {
    bool operator()(const Index& a, const Index& b) const {
        return a.cost > b.cost;
    }
};


class AStarPlanner{
    public:
        AStarPlanner(tf::TransformListener& tf);
        ~AStarPlanner();
        //获取地图，做出规划
        bool makePlan(const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped goal, std::vector<geometry_msgs::PoseStamped>& plan);
        //计算potentials
        //costs:map的值
        bool calculatePotentials(int* map, double start_x, double start_y, double end_x, double end_y, int cycles, float* potential);
        bool getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                       std::vector<geometry_msgs::PoseStamped>& plan);
        bool getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path);
        bool PointValid(geometry_msgs::PoseStamped point);//判断目标点所在栅格是否处于障碍物中或者未知
        void publishPlan(const std::vector<geometry_msgs::PoseStamped>& path);
        void publishPotential(float* potential);
        bool getRobotPose(tf::Stamped<tf::Pose>& global_pose) const;//获取机器人位置
    private:
        void add(int* map, float* potential, float prev_potential, int next_i, int end_x, int end_y);
        std::vector<Index> queue_;
        int nx_, ny_, ns_;//map的大小
        double resolution_;
        StaticMap* sm;//静态地图对象
        float* potential_array_;
        ros::Publisher plan_pub_;
        
        //tf相关
        tf::TransformListener& tf_;
        std::string global_frame_id, robot_base_frame_id;
        double transform_tolerance_;
};
}





#endif