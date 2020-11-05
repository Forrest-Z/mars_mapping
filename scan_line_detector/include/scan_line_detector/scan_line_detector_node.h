#ifndef SCAN_LINE_DETECTOR_NODE_H
#define SCAN_LINE_DETECTOR_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/Marker.h>
#include "scan_line_detector/laserscan_processor.h"
#include "scan_line_detector/map_manager.h"
#include "mars_srvs/DetectLine.h"

class ScanLineDetector
{
public:
    explicit ScanLineDetector();
    ScanLineDetector(const ScanLineDetector&) = delete;
    ~ScanLineDetector(){}
    void laserScanCallBack(const sensor_msgs::LaserScanConstPtr& msg);

private:
    void initLinesMarker();
    void callDetectLine(const nav_msgs::OccupancyGridPtr& map);
    void pubLines(const hough_line_msgs::Lines& lines);

    std::shared_ptr<ros::NodeHandle> nh;

    std::unique_ptr<LaserscanProcessor> laserscan_processor;
    std::unique_ptr<MapManager> map_manager;

    std::string scan_topic;
    
    ros::Subscriber scan_sub;
    ros::Publisher marker_pub;

    visualization_msgs::Marker lines_marker;
};

#endif 