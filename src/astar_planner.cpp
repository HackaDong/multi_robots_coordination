#include <multi_robots_coordination/astar_planner.h>
#include <nav_msgs/Path.h>

namespace multi_robots_coordination{
    
AStarPlanner::AStarPlanner(tf::TransformListener& tf):
tf_(tf)
{
    sm = new StaticMap();
    global_frame_id = "/map";
    robot_base_frame_id = "/base_link";
    nx_ = sm->getSizeInCellsX();
    ny_ = sm->getSizeInCellsY();
    resolution_ = sm->getResolution();
    ns_ = nx_ * ny_;

    ros::NodeHandle nh;
    plan_pub_ = nh.advertise<nav_msgs::Path>("path", 1);

    //TODO：需要加上TF_prefix解析，分robot1和robot2
    //两台机器人的tf_prefix是什么样的，未知

    // // get our tf prefix
    // ros::NodeHandle prefix_nh;
    // std::string tf_prefix = tf::getPrefixParam(prefix_nh);

    // // get two frames
    // private_nh.param("global_frame", global_frame_, std::string("/map"));
    // private_nh.param("robot_base_frame", robot_base_frame_, std::string("base_link"));

    // // make sure that we set the frames appropriately based on the tf_prefix
    // global_frame_ = tf::resolve(tf_prefix, global_frame_);
    // robot_base_frame_ = tf::resolve(tf_prefix, robot_base_frame_);



    printf("i'm here\n");
}

AStarPlanner::~AStarPlanner(){

}

bool AStarPlanner::makePlan(const geometry_msgs::PoseStamped start, const geometry_msgs::PoseStamped goal,
                           std::vector<geometry_msgs::PoseStamped>& plan) {
    //判断目标点所在栅格是否处于障碍物中或者未知
    if(!PointValid(start)){
        printf("Error Start Point\n");
        return false;
    }  
    if(!PointValid(goal)){
        printf("Error End Point\n");
        return false;
    }   
    
    //clear the plan, just in case
    plan.clear();

    ros::NodeHandle n;
    std::string global_frame = global_frame_id;

    double wx = start.pose.position.x;
    double wy = start.pose.position.y;

    double start_x, start_y, goal_x, goal_y;

    sm->worldToMap(wx, wy, start_x, start_y);

    wx = goal.pose.position.x;
    wy = goal.pose.position.y;

    sm->worldToMap(wx, wy, goal_x, goal_y);

    //分配空间，大小和costmap一样大
    potential_array_ = new float[nx_ * ny_];//相当于开辟一个二维空间

    //a星调用
    bool found_legal = calculatePotentials(sm->getMap(), start_x, start_y, goal_x, goal_y,
                                                    nx_ * ny_ * 2, potential_array_);
    
    for(int i; i < nx_ * ny_; ++i){
        if(potential_array_[i] < 255){
            printf("potential %d: %f\n", i, potential_array_[i]);
        }   
    }  

    if (found_legal) {
        //extract the plan
        if (getPlanFromPotential(start_x, start_y, goal_x, goal_y, plan)) {
            //make sure the goal we push on has the same timestamp as the rest of the plan
            geometry_msgs::PoseStamped goal_copy = goal;
            goal_copy.header.stamp = ros::Time::now();
            plan.push_back(goal_copy);
        } else {
            ROS_ERROR("Failed to get a plan from potential when a legal potential was found. This shouldn't happen.");
        }
    }else{
        ROS_ERROR("Failed to get a plan.");
    }

    // while(1){
    //     publishPlan(plan);
    //     //printf("break~\n");
    // }
    
    delete potential_array_;
    return !plan.empty();
}


bool AStarPlanner::calculatePotentials(int* map, double start_x, double start_y, double end_x, double end_y,
                                        int cycles, float* potential) {
    queue_.clear();
    int start_i = sm->toIndex(start_x, start_y);
    queue_.push_back(Index(start_i, 0));//Index对象，起始点start_i，cost=0

    std::fill(potential, potential + ns_, POT_HIGH);//将所有栅格的potential值赋最大值
    potential[start_i] = 0;

    int goal_i = sm->toIndex(end_x, end_y);
    int cycle = 0;

    //判断下个点移动代价值
    //直线移动代价小设为1，折线设为2

    while (queue_.size() > 0 && cycle < cycles) {
        // printf("cycle: %d\n", cycle);
        // for(int i = 0; i < queue_.size(); ++i)  
        //     printf("i: %d, cost: %d\n", queue_[i].i, int(queue_[i].cost));
        // printf("\n");

        Index top = queue_[0];
        std::pop_heap(queue_.begin(), queue_.end(), greater1());
        queue_.pop_back();

        // for(int i = 0; i < queue_.size(); ++i)  
        //     printf("i: %d, cost: %d\n", queue_[i].i, int(queue_[i].cost));
        // printf("\n");

        int i = top.i;
        if (i == goal_i)
            return true;

        add(map, potential, potential[i], i + 1, end_x, end_y);
        add(map, potential, potential[i], i - 1, end_x, end_y);
        add(map, potential, potential[i], i + nx_, end_x, end_y);
        add(map, potential, potential[i], i - nx_, end_x, end_y);

        cycle++;
    }

    return false;
}

void AStarPlanner::add(int* map, float* potential, float prev_potential, int next_i, int end_x, int end_y){
    float costs;//地图的代价值

    if (next_i < 0 || next_i >= ns_)
        return;

    if (potential[next_i] < POT_HIGH)//next不是POT_HIGH，表示是已经路过过的点
        return;

    if(map[next_i] != 0)//非0表示不是可通行栅格
        costs = POT_HIGH;
    else
        costs = 1.0;
        
    potential[next_i] = costs + prev_potential;//计算
    int x = next_i % nx_, y = next_i / nx_;
    float distance = abs(end_x - x) + abs(end_y - y);

    queue_.push_back(Index(next_i, potential[next_i] + distance));
    std::push_heap(queue_.begin(), queue_.end(), greater1());
}

bool AStarPlanner::getPlanFromPotential(double start_x, double start_y, double goal_x, double goal_y,
                                       std::vector<geometry_msgs::PoseStamped>& plan) {
    std::string global_frame = global_frame_id;

    //clear the plan, just in case
    plan.clear();

    std::vector<std::pair<float, float> > path;

    if (!getPath(potential_array_, start_x, start_y, goal_x, goal_y, path)) {
        ROS_ERROR("NO PATH!");
        return false;
    }

    ros::Time plan_time = ros::Time::now();
    for (int i = path.size() -1; i>=0; i--) {
        std::pair<float, float> point = path[i];
        //convert the plan to world coordinates
        double world_x, world_y;
        sm->mapToWorld(point.first, point.second, world_x, world_y);

        geometry_msgs::PoseStamped pose;
        pose.header.stamp = plan_time;
        pose.header.frame_id = global_frame;
        pose.pose.position.x = world_x;
        pose.pose.position.y = world_y;
        pose.pose.position.z = 0.0;
        pose.pose.orientation.x = 0.0;
        pose.pose.orientation.y = 0.0;
        pose.pose.orientation.z = 0.0;
        pose.pose.orientation.w = 1.0;
        plan.push_back(pose);
    }
    
    return !plan.empty();
}


//根据计算完的potential寻找最优路径
bool AStarPlanner::getPath(float* potential, double start_x, double start_y, double end_x, double end_y, std::vector<std::pair<float, float> >& path) {
    std::pair<float, float> current;
    current.first = end_x;
    current.second = end_y;

    int start_index = sm->toIndex(start_x, start_y);

    path.push_back(current);
    int c = 0;
    
    while (sm->toIndex(current.first, current.second) != start_index) {
        float min_val = 1e10;
        int min_x = 0, min_y = 0;
        for (int xd = -1; xd <= 1; xd++) {
            for (int yd = -1; yd <= 1; yd++) {
                if (xd == 0 && yd == 0)
                    continue;
                int x = current.first + xd, y = current.second + yd;
                int index = sm->toIndex(x, y);
                if (potential[index] < min_val) {
                    min_val = potential[index];
                    min_x = x;
                    min_y = y;
                }
            }
        }
        if (min_x == 0 && min_y == 0)
            return false;
        current.first = min_x;
        current.second = min_y;
        path.push_back(current);
        
        if(c++>ns_*4){
            return false;
        }

    }
    return true;
}

void AStarPlanner::publishPlan(const std::vector<geometry_msgs::PoseStamped>& path) {
    //create a message for the plan
    nav_msgs::Path gui_path;
    gui_path.poses.resize(path.size());

    gui_path.header.frame_id = global_frame_id;
    gui_path.header.stamp = ros::Time::now();

    // Extract the plan in world co-ordinates, we assume the path is all in the same frame
    for (unsigned int i = 0; i < path.size(); i++) {
        gui_path.poses[i] = path[i];
    }

    plan_pub_.publish(gui_path);
}

//判断目标点所在栅格是否处于障碍物中或者未知
bool AStarPlanner::PointValid(geometry_msgs::PoseStamped point){
    double wx, wy;
    sm->worldToMap(point.pose.position.x, point.pose.position.y, wx, wy);
    if(wx < 0 || wx > nx_ || wy < 0 || wy > ny_)  return false;
    if(sm->getMap()[sm->toIndex(wx, wy)] != 0)    return false;
    return true;
}

//获取机器人位置
bool AStarPlanner::getRobotPose(tf::Stamped<tf::Pose>& global_pose) const
{
    //TODO：需要加上TF_prefix解析，分robot1和robot2
    global_pose.setIdentity();
    tf::Stamped < tf::Pose > robot_pose;
    robot_pose.setIdentity();
    robot_pose.frame_id_ = robot_base_frame_id;
    robot_pose.stamp_ = ros::Time();
    ros::Time current_time = ros::Time::now();  // save time for checking tf delay later

    // get the global pose of the robot
    try
    {
        tf_.transformPose(global_frame_id, robot_pose, global_pose);
    }
    catch (tf::LookupException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "No Transform available Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf::ConnectivityException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Connectivity Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    catch (tf::ExtrapolationException& ex)
    {
        ROS_ERROR_THROTTLE(1.0, "Extrapolation Error looking up robot pose: %s\n", ex.what());
        return false;
    }
    // check global_pose timeout
    if (current_time.toSec() - global_pose.stamp_.toSec() > transform_tolerance_)
    {
        ROS_WARN_THROTTLE(1.0,
                        "Costmap2DROS transform timeout. Current time: %.4f, global_pose stamp: %.4f, tolerance: %.4f",
                        current_time.toSec(), global_pose.stamp_.toSec(), transform_tolerance_);
        return false;
    }

    return true;
}


// void AStarPlanner::publishPotential(float* potential)
// {
//     nav_msgs::OccupancyGrid grid;
//     // Publish Whole Grid
//     grid.header.frame_id = frame_id;
//     grid.header.stamp = ros::Time::now();
//     grid.info.resolution = resolution_;

//     grid.info.width = nx_;
//     grid.info.height = ny_;

//     double wx, wy;
//     sm->mapToWorld(0, 0, wx, wy);
//     grid.info.origin.position.x = wx - resolution / 2;
//     grid.info.origin.position.y = wy - resolution / 2;
//     grid.info.origin.position.z = 0.0;
//     grid.info.origin.orientation.w = 1.0;

//     grid.data.resize(nx_ * ny_);

//     float max = 0.0;
//     for (unsigned int i = 0; i < grid.data.size(); i++) {
//         float potential = potential_array_[i];
//         if (potential < POT_HIGH) {
//             if (potential > max) {
//                 max = potential;
//             }
//         }
//     }

//     for (unsigned int i = 0; i < grid.data.size(); i++) {
//         if (potential_array_[i] >= POT_HIGH) {
//             grid.data[i] = -1;
//         } else
//             grid.data[i] = potential_array_[i] * publish_scale_ / max;
//     }
//     potential_pub_.publish(grid);
// }

}
