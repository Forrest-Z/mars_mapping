#ifndef OCC_LINE_DETECTOR_H
#define OCC_LINE_DETECTOR_H

#include <time.h>
#include <omp.h>
#include <ros/ros.h>
#include <nav_msgs/SetMap.h>
#include "hough_line_msgs/Lines.h"
#include "mars_srvs/DetectLine.h"

class OccLineDetector
{
public:
    explicit OccLineDetector();
    OccLineDetector(const OccLineDetector&) = delete;
    ~OccLineDetector(){}
    bool lineDetection(mars_srvs::DetectLine::Request& req,mars_srvs::DetectLine::Response& res);
    

private:
    bool checkRepeat(const hough_line_msgs::Line& new_line,hough_line_msgs::Lines& lines);
    void inflateMap(const nav_msgs::OccupancyGrid& origin_map,nav_msgs::OccupancyGridPtr& new_map);

    std::shared_ptr<ros::NodeHandle> nh;
    ros::ServiceServer set_map_server;
    ros::Publisher inflate_map_pub;
    ros::Publisher hough_map_pub;

    // param
    int omp_max_thread_num;
    double angle_resolution,r_resolution,angle_window_size,r_window_size; 
    double parellel_threshold,r_window_size_for_parallel;
    int line_threshold;
    bool inflate_map,pub_map;
};

#endif 