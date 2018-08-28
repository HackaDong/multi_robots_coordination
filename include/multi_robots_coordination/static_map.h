#ifndef STATIC_MAP_H
#define STATIC_MAP_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/GetMap.h>

#define MAX_COST 255

namespace multi_robots_coordination{
class StaticMap{
    public:
        StaticMap();
        ~StaticMap();
        bool worldToMap(double wx, double wy, double& mx, double& my);
        void mapToWorld(double mx, double my, double& wx, double& wy);

        void resizeMap();

        /**
        * @brief  Accessor for the x size of the costmap in cells
        * @return The x size of the costmap
        */
        unsigned int getSizeInCellsX() const;
        /**
        * @brief  Accessor for the y size of the costmap in cells
        * @return The y size of the costmap
        */
        unsigned int getSizeInCellsY() const;
        /**
        * @brief  Will return a pointer to the underlying unsigned char array used as the costmap
        * @return A pointer to the underlying unsigned char array storing cost values
        */
        int* getMap() const;
        double getOriginX();
        double getOriginY();
        double getResolution();

        inline int toIndex(int x, int y) {
            return x + nx_ * y;
        }//将x,y坐标形式转成index序号

    private:
        int nx_, ny_;//map大小
        double resolution_;//分辨率
        double origin_x_, origin_y_;//起始点地图坐标
        int* map_data_;//储存数据
        bool map_received_;
        //ros::Subscriber map_sub_;
        ros::ServiceClient map_client_;
        nav_msgs::OccupancyGrid map_origin_;
        nav_msgs::GetMap srv;
};
}


#endif