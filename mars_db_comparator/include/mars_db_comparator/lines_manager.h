#ifndef LINES_MANAGER_H
#define LINES_MANAGER_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <hough_line_msgs/Lines.h>

class LinesManager
{
public:
    explicit LinesManager(){}
    explicit LinesManager(ros::NodeHandle& nh_);
    LinesManager(const LinesManager&) = delete;
    ~LinesManager(){}

private:
    void loadLinesFile(const std::string& file_path);
    std::stringstream getSstream(std::ifstream& file);

    ros::NodeHandle nh;

    std::string lines_file_path;

    int submap_num;
    std::vector<hough_line_msgs::Lines> submap_lines_array;
};

#endif