#include "db_line_detector_node.h"

DBLineDetector::DBLineDetector()
{
    nh = std::make_shared<ros::NodeHandle>("~");
    map_manager = std::make_unique<MapManager>(*nh);

    load_map_service = nh->advertiseService("load_map",&DBLineDetector::callLoadMap,this);
    detect_line_service = nh->advertiseService("detect_line",&DBLineDetector::callDetectLine,this);
}

bool DBLineDetector::callLoadMap(mars_srvs::LoadMap::Request& req,mars_srvs::LoadMap::Response& res)
{
    if(map_manager->loadMap(req.map_path))
        return true;
    else
        return false;
}

bool DBLineDetector::callDetectLine(std_srvs::Empty::Request& req,std_srvs::Empty::Response& res)
{

    if(map_manager->detectLine())
        return true;
    else
        return false;
}