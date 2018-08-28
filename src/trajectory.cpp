#include <multi_robots_coordination/trajectory.h>
#include <math.h>

namespace multi_robots_coordination{
    Trajectory::Trajectory(){
        
    }

    Trajectory::~Trajectory(){

    }

    bool Trajectory::makePlan(geometry_msgs::Pose current_pose, std::vector<geometry_msgs::Pose> g, std::vector< std::vector<geometry_msgs::Pose> >& traj, double s){
        //makePlan前清空容器
        trajectory.clear();
        goals.clear();
        //目标点向量为空，返回false
        if(!g.size()){
            ROS_INFO("Empty goals!");
            return false;
        }

        goals = g;//目标点
        step_dis = s;//步长
        int num = goals.size();//目标点个数
        int one_point = 0;//是否单个目标点
        if(num == 1)    one_point = 1;
        //如果单个点
        if(one_point){
            //处理第一个目标点
            traj.push_back(createTrajectory(current_pose, goals[0], current_pose));
            trajectory.push_back(createTrajectory(current_pose, goals[0], current_pose));
            if(traj.size() == 0 || traj[0].size() == 0) return false;
            int t_s = trajectory.size();
            printf("1 trajectory needs to be generated, %d have been done!\n", t_s);
            return true;
        }
        else{
            //多点情况
            //处理第一个目标点
            traj.push_back(createTrajectory(current_pose, goals[0], goals[1]));
            trajectory.push_back(createTrajectory(current_pose, goals[0], goals[1]));
            if(traj.size() == 0 || traj[0].size() == 0) return false;
            //对剩余每个目标点进行规划,除了最后一个点
            for(int i = 1; i < (num - 1); ++i){
                traj.push_back(createTrajectory(goals[i-1], goals[i], goals[i+1]));
                trajectory.push_back(createTrajectory(goals[i-1], goals[i], goals[i+1]));
                if(traj.size() == 0 || traj[i].size() == 0) return false;
            }
            //对最后一个点进行处理
            traj.push_back(createTrajectory(goals[num - 2], goals.back(), goals[num - 2]));
            trajectory.push_back(createTrajectory(goals[num - 2], goals.back(), goals[num - 2]));
            if(traj.size() == 0 || traj[num-1].size() == 0) return false;
            //打印
            int t_s = trajectory.size();
            printf("%d trajectories need to be generated, %d have been done!\n", num, t_s);
            // int tra_size = trajectory.size();
            // printf("size of trajectory = %d\n", tra_size);
            //输出轨迹
            for(int i = 0; i < trajectory.size(); i++){
                for(int j = 0; j < trajectory[i].size(); j++){
                    printf("x = %f, y = %f, theta = %f \n", trajectory[i][j].position.x, trajectory[i][j].position.y, quaternionToEuler(trajectory[i][j].orientation).z);
                }
            }
            return true;
            //生成完毕
        }
        return false;
    }

    bool Trajectory::getPositionAt(int index1, int index2, geometry_msgs::Pose& pose){
        if( index1 >= trajectory.size() )  return false;//如果轨迹标识大于总轨技数
        if( index2 < trajectory[index1].size() ){//判断index2是否超过当前轨迹的总点数
            pose = trajectory[index1][index2];
        }
        else{
            pose = trajectory[index1][trajectory[index1].size() - 1];//返回最后的目标点
        }
        return true;
    }

    double Trajectory::calDistance(double x, double y){
        return sqrt(x*x + y*y);
    }

    geometry_msgs::Quaternion Trajectory::getOrientation(geometry_msgs::Pose p1, geometry_msgs::Pose p2){
        geometry_msgs::Vector3 vec;
        vec.z = atan2(p2.position.y - p1.position.y, p2.position.x - p1.position.x);
        return eulerToQuaterion(vec);

    }

    std::vector<geometry_msgs::Pose> Trajectory::createTrajectory(geometry_msgs::Pose begin, geometry_msgs::Pose end, geometry_msgs::Pose end_n){
        geometry_msgs::Pose pose_tmp;
        std::vector<geometry_msgs::Pose> traj_tmp;
        double x_0 = 0, y_0 = 0, delta_x = 0, delta_y = 0;
        x_0 = begin.position.x;
        y_0 = begin.position.y;
        delta_x = end.position.x - x_0;
        delta_y = end.position.y - y_0;
        int steps = calDistance(delta_x, delta_y) / step_dis;//总步数
        //计算一个路径上的每个点
        for(int i = 0; i < steps; ++i){
            pose_tmp.position.x = x_0 + i * delta_x / steps;
            pose_tmp.position.y = y_0 + i * delta_y / steps;
            pose_tmp.orientation.w = 1;
            traj_tmp.push_back(pose_tmp);
        }
        pose_tmp.position = end.position;
        pose_tmp.orientation = getOrientation(end, end_n);//计算当前点和下个点的连线的角度，设为最后一个点的orientaiton
        traj_tmp.push_back(pose_tmp);

        return traj_tmp;
    }

    geometry_msgs::Vector3 Trajectory::quaternionToEuler(geometry_msgs::Quaternion q){
        //printf("quaternion x = %f, y = %f, z = %f, w = %f\n", q.x, q.y, q.z, q.w);
        tf::Quaternion quat;
        tf::quaternionMsgToTF(q, quat);

        // the tf::Quaternion has a method to acess roll pitch and yaw
        double roll, pitch, yaw;
        tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);

        // the found angles are written in a geometry_msgs::Vector3
        geometry_msgs::Vector3 rpy;
        rpy.x = roll;
        rpy.y = pitch;
        rpy.z = yaw;

        return rpy;
    }

    geometry_msgs::Quaternion Trajectory::eulerToQuaterion(geometry_msgs::Vector3 v){
        return tf::createQuaternionMsgFromRollPitchYaw(v.x, v.y, v.z);
    }

    //获取该条路径的所有点数
    int Trajectory::getSteps(int index){
        return trajectory[index].size();
    }

    bool Trajectory::goalReached(int traj_index, geometry_msgs::Pose current_pose, double pos_tolerance){
        
        double current_pose_x = current_pose.position.x;
        double current_pose_y = current_pose.position.y;
        double goal_pose_x = trajectory[traj_index].back().position.x;
        // for(int i = 0; i < trajectory[traj_index].size(); i++){
        //     printf("x = %f, y = %f\n", trajectory[traj_index][i].position.x, trajectory[traj_index][i].position.y);
        // }
        double goal_pose_y = trajectory[traj_index].back().position.y;
        // printf("delta = %f\n", pow(current_pose_x - goal_pose_x, 2) + pow(current_pose_y - goal_pose_y, 2));
        if( pow(current_pose_x - goal_pose_x, 2) + pow(current_pose_y - goal_pose_y, 2) < pow(pos_tolerance, 2) ){
            return true;
        }
        else{
            return false;
        }
    }

    bool Trajectory::OrientationReached(int traj_index, geometry_msgs::Pose current_pose, double pos_tolerance, double &angle_diff){
        double current_yaw = quaternionToEuler(current_pose.orientation).z;
        double goal_yaw = quaternionToEuler(trajectory[traj_index].back().orientation).z;
        angle_diff = angleDiff(current_yaw, goal_yaw);
        printf("angle_diff = %f\n", angle_diff);
        if(abs(angle_diff) < pos_tolerance) return true;
        else return false;
    }

    double Trajectory::normalize(double z){
        return atan2(sin(z),cos(z));
    }

    double Trajectory::angleDiff(double a, double b)
    {
        double d1, d2;
        a = normalize(a);
        b = normalize(b);
        d1 = a-b;
        d2 = 2*M_PI - fabs(d1);
        if(d1 > 0)
            d2 *= -1.0;
        if(fabs(d1) < fabs(d2))
            return(d1);
        else
            return(d2);
    }
};
