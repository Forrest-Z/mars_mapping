#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

class MapManager
{
public:
    explicit MapManager(){}
    explicit MapManager(ros::NodeHandle& nh_);
    ~MapManager(){}
    MapManager(const MapManager&) = delete;

private:
    void loadMap(const std::string& file_path);
    std::stringstream getSstream(std::ifstream& file);
    nav_msgs::OccupancyGridPtr readMap(std::ifstream& file);

    ros::NodeHandle nh;
    std::string db_file_path;

    int submap_num;
    double resolution;
    std::vector<nav_msgs::OccupancyGridPtr> submaps;
};

#endif 