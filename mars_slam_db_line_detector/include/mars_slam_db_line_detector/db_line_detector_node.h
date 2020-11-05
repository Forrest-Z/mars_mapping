#ifndef DB_LINE_DETECTOR_NODE_H
#define DB_LINE_DETECTOR_NODE_H

#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include "map_manager.h"
#include "mars_srvs/LoadMap.h"

class DBLineDetector
{
public:
    explicit DBLineDetector();
    DBLineDetector(const DBLineDetector&) = delete;
    ~DBLineDetector(){}
    bool callLoadMap(mars_srvs::LoadMap::Request& req,mars_srvs::LoadMap::Response& res);
    bool callDetectLine(std_srvs::Empty::Request& req,std_srvs::Empty::Response& res);

private:
    std::shared_ptr<ros::NodeHandle> nh;
    ros::ServiceServer load_map_service;
    ros::ServiceServer detect_line_service;

    std::unique_ptr<MapManager> map_manager;

};

#endif 