#include <multi_robots_coordination/static_map.h>
#include <boost/algorithm/string.hpp>
#include <boost/thread.hpp>


namespace multi_robots_coordination{

StaticMap::StaticMap(){
    ros::NodeHandle nh;
    map_client_ = nh.serviceClient<nav_msgs::GetMap>("static_map");
    map_client_.call(srv);
    if(srv.response.map.info.resolution){
        map_origin_ = srv.response.map;
        ROS_INFO("hello");

        nx_ = map_origin_.info.width;
        ny_ = map_origin_.info.height;
        resolution_ = map_origin_.info.resolution;
        origin_x_ = map_origin_.info.origin.position.x;
        origin_y_ = map_origin_.info.origin.position.y;

        ROS_INFO("Received a %d X %d map at %f m/pix", nx_, ny_, resolution_);

        resizeMap();

        ROS_INFO("Resized a %d X %d map at %f m/pix", nx_, ny_, resolution_);
        ROS_INFO("origin_x: %f, origin_y: %f", origin_x_, origin_y_);

        map_received_ = true;
    }
    else{
        ROS_INFO("No map received!");
    }

}

StaticMap::~StaticMap(){
    delete map_data_;
}


void StaticMap::resizeMap(){
    //缩减地图，分别定位到已知点中四个方向最远处
    unsigned int up = 0, down = ny_, left = nx_, right = 0;
    for (unsigned int i = 0; i < ny_; ++i)
    {
        for (unsigned int j = 0; j < nx_; ++j)
        {
            unsigned int value = map_origin_.data[toIndex(j, i)];
            //ROS_INFO("x: %d, y: %d, index = %d, data = %d", j, i, toIndex(j, i), value);
            if(value != -1){//判断为有效点,i代表行数，j代表列数
                if(i > up)      up = i;
                if(i < down)    down = i;
                if(j < left)    left = j;
                if(j > right)   right = j;
            }
        }
    }
    ROS_INFO("up: %d, down: %d, left: %d, right: %d", up, down, left, right);
    
    int nx_tmp = right - left + 1;
    int ny_tmp = up - down + 1;

    map_data_ = new int[ny_tmp * nx_tmp];//相当于开辟一个二维空间
    unsigned int index = 0;

    //重新赋值map
    for (unsigned int i = down; i < up + 1; ++i)
    {
        for (unsigned int j = left; j < right + 1; ++j)
        {
            int value = map_origin_.data[toIndex(j, i)];
            printf("%d ", value);
            map_data_[index++] = value;
        }
    }

    //更新参数
    nx_ = nx_tmp;
    ny_ = ny_tmp;
    double ox, oy;
    mapToWorld(left, down, ox, oy);
    origin_x_ = ox;
    origin_y_ = oy;

    // for(unsigned int i = 0; i < nx_ * ny_; i++){
    //     printf("%d ", map_data_[i]);
    // }
}

//将点坐标转成map的栅格坐标
bool StaticMap::worldToMap(double wx, double wy, double& mx, double& my){
    if(!map_received_){
        ROS_DEBUG("MAP didn't received!\n");
        return false;
    }

    if (wx < origin_x_ || wy < origin_y_)
        return false;

    mx = (wx - origin_x_) / resolution_;
    my = (wy - origin_y_) / resolution_;

    if (mx < nx_ && my < ny_)
        return true;

    return false;
}

void StaticMap::mapToWorld(double mx, double my, double& wx, double& wy){
    wx = origin_x_ + mx * resolution_;
    wy = origin_y_ + my * resolution_;
}

unsigned int StaticMap::getSizeInCellsX() const{
    return nx_;
}

unsigned int StaticMap::getSizeInCellsY() const{
    return ny_;
}
        
int* StaticMap::getMap() const{
    return map_data_;
}

double StaticMap::getOriginX(){
    return origin_x_;
}

double StaticMap::getOriginY(){
    return origin_y_;
}

double StaticMap::getResolution(){
    return resolution_;
}

}