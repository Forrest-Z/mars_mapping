#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>

typedef struct
{
    double max_x;
    double max_y;
    double min_x;
    double min_y;
}MaxMin;

class MapManager
{
public:
    explicit MapManager(){}
    explicit MapManager(ros::NodeHandle& nh_);
    MapManager(const MapManager&) = delete;
    ~MapManager(){}
    void transferPointCloudToOccMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                          nav_msgs::OccupancyGridPtr& map);

private:

    void setMapInfo(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          nav_msgs::OccupancyGridPtr& map);
    MaxMin findCloudMaxMin(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void setMapData(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          nav_msgs::OccupancyGridPtr& map);

    ros::NodeHandle nh;
    ros::Publisher map_pub;
    
    double resolution;

};

#endif