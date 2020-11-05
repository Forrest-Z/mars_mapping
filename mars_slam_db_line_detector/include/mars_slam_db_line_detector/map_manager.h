#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include <iostream>
#include <fstream>
#include <sstream>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_datatypes.h>
#include "mars_srvs/DetectLine.h"

class MapManager
{
public:
    explicit MapManager(){}
    explicit MapManager(ros::NodeHandle& nh_);
    ~MapManager(){}
    MapManager(const MapManager&) = delete;
    bool loadMap(const std::string& file_path);
    bool detectLine();

private:
    void initLinesMarker();
    std::stringstream getSstream(std::ifstream& file);
    nav_msgs::OccupancyGridPtr readMap(std::ifstream& file);
    void showLines();
    void saveLines();
    std_msgs::ColorRGBA green();
    std_msgs::ColorRGBA blue();
    std_msgs::ColorRGBA orange();
    std_msgs::ColorRGBA purple();
    std_msgs::ColorRGBA yellow();


    ros::NodeHandle nh;

    std::vector<nav_msgs::OccupancyGridPtr> submaps;
    std::vector<ros::Publisher> submaps_pub_array;
    std::vector<hough_line_msgs::Lines> submap_lines_array;
    nav_msgs::OccupancyGridPtr map;
    ros::Publisher map_pub;
    std::string map_file_path;

    int submap_num;
    double resolution;
    bool map_received;

    visualization_msgs::MarkerArray lines_marker_array;
    visualization_msgs::Marker lines_marker;
    ros::Publisher lines_marker_pub;

};

#endif