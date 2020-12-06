#ifndef DB_COMPARATOR_NODE_H
#define DB_COMPARATOR_NODE_H

#include <time.h>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include <visualization_msgs/Marker.h>
#include "mars_db_comparator/map_manager.h"
#include "mars_db_comparator/lines_manager.h"
#include "mars_db_comparator/laserscan_processor.h"
#include "mars_srvs/GlobalLocalization.h"

class DBComparator
{
public:
    explicit DBComparator();
    ~DBComparator(){}
    DBComparator(const DBComparator&) = delete;
    void laserScanCallBack(const sensor_msgs::LaserScanConstPtr& msg);
    bool callCompare(std_srvs::Empty::Request& req,std_srvs::Empty::Response& res);

private:
    void callDetectLine(const nav_msgs::OccupancyGridPtr& map);
    void initLinesMarker();
    void pubLines(const hough_line_msgs::Lines& lines);

    std::shared_ptr<ros::NodeHandle> nh;
    ros::ServiceServer compare_db_service;
    ros::Publisher marker_pub;

    std::unique_ptr<MapManager> map_manager;
    std::unique_ptr<LinesManager> lines_manager;
    std::unique_ptr<LaserscanProcessor> laserscan_processor;

    std::string scan_topic;
    ros::Subscriber scan_sub;
    sensor_msgs::LaserScan last_scan;

    visualization_msgs::Marker lines_marker;
};


#endif 