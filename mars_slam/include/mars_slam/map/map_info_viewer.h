#ifndef MAP_INFO_VIEWER_H  
#define MAP_INFO_VIEWER_H  

#include <ros/ros.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_datatypes.h>
#include "mars_slam/map/occ_map.h"

class MapInfoViewer
{
public:
    explicit MapInfoViewer(){}
    explicit MapInfoViewer(ros::NodeHandle& nh_);
    MapInfoViewer(const MapInfoViewer&) = delete;
    ~MapInfoViewer(){}
    void updateSubmapCenters(std::vector<std::shared_ptr<OccMap>>& submaps);

private:
    void initNumMarker();
    std_msgs::ColorRGBA red();
    std_msgs::ColorRGBA green();
    std_msgs::ColorRGBA blue();
    std_msgs::ColorRGBA yellow();
    std_msgs::ColorRGBA orange();
    std_msgs::ColorRGBA purple();
    
    ros::NodeHandle nh;

    // view objects
    geometry_msgs::PoseArray submap_center_array;
    visualization_msgs::Marker submap_num_marker;
    visualization_msgs::MarkerArray submap_num_marker_array;

    // publishers
    ros::Publisher centers_pub;
    ros::Publisher num_markers_pub;
};

#endif 