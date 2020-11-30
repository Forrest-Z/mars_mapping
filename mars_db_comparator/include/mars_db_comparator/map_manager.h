#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/SetMap.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <tf/transform_datatypes.h>
#include "mars_srvs/DetectLine.h"

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
    ~MapManager(){}
    MapManager(const MapManager&) = delete;
    void transferPointCloudToOccMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                          nav_msgs::OccupancyGridPtr& map);
    const nav_msgs::OccupancyGridPtr getMap(const int& n){return submaps[n];} 
    void pubMap(const int& n);

private:
    void loadMap(const std::string& file_path);
    std::stringstream getSstream(std::ifstream& file);
    nav_msgs::OccupancyGridPtr readMap(std::ifstream& file);
    void setMapInfo(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          nav_msgs::OccupancyGridPtr& map);
    MaxMin findCloudMaxMin(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud);
    void setMapData(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                          nav_msgs::OccupancyGridPtr& map);

    ros::NodeHandle nh;
    ros::Publisher map_pub;
    std::string db_file_path;

    int submap_num;
    double resolution;
    std::vector<nav_msgs::OccupancyGridPtr> submaps;

    // scan map
    ros::Publisher scan_map_pub;
    double scan_map_resolution;
};

#endif 