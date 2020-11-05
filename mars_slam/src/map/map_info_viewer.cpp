#include "mars_slam/map/map_info_viewer.h"

MapInfoViewer::MapInfoViewer(ros::NodeHandle& nh_):nh(nh_)
{
    centers_pub = nh.advertise<geometry_msgs::PoseArray>("centers",1,true);
    num_markers_pub = nh.advertise<visualization_msgs::MarkerArray>("serial_nums",1,true);

    submap_center_array.header.frame_id = "map";
    submap_center_array.header.stamp = ros::Time();

    initNumMarker();
}

void MapInfoViewer::updateSubmapCenters(std::vector<std::shared_ptr<OccMap>>& submaps)
{
    submap_center_array.poses.clear();
    submap_center_array.poses.reserve(submaps.size());
    submap_num_marker_array.markers.clear();
    submap_num_marker_array.markers.reserve(submaps.size());

    for(unsigned int i = 0;i < submaps.size();i++)
    {
        geometry_msgs::Pose pose;
        tf::poseTFToMsg(submaps[i]->getCenter(),pose);

        // pose array
        submap_center_array.poses.push_back(pose);

        // num marker
        submap_num_marker.pose = pose;
        submap_num_marker.pose.position.x += 0.5;
        submap_num_marker.pose.position.y += 0.5;
        submap_num_marker.pose.position.z = 0.5;
        submap_num_marker.id = i;
        submap_num_marker.text = std::to_string(submap_num_marker.id);
        submap_num_marker_array.markers.push_back(submap_num_marker);
    }

    centers_pub.publish(submap_center_array);
    num_markers_pub.publish(submap_num_marker_array);
}

void MapInfoViewer::initNumMarker()
{
    submap_num_marker.header.frame_id = "map";
    submap_num_marker.header.stamp = ros::Time();
    submap_num_marker.ns = "";
    submap_num_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    submap_num_marker.action = visualization_msgs::Marker::ADD;
    submap_num_marker.scale.z = 0.4;
    submap_num_marker.color = red();
}

std_msgs::ColorRGBA MapInfoViewer::red()
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;color.r = 1.0;color.g = 0.0;color.b = 0.0;
    return color;
}

std_msgs::ColorRGBA MapInfoViewer::green()
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;color.r = 0.0;color.g = 1.0;color.b = 0.0;
    return color;
}

std_msgs::ColorRGBA MapInfoViewer::blue()
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;color.r = 0.0;color.g = 0.0;color.b = 1.0;
    return color;
}

std_msgs::ColorRGBA MapInfoViewer::yellow()
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;color.r = 1.0;color.g = 0.843;color.b = 0.0;
    return color;
}

std_msgs::ColorRGBA MapInfoViewer::orange()
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;color.r = 1;color.g = 0.5;color.b = 0.0;
    return color;
}

std_msgs::ColorRGBA MapInfoViewer::purple()
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;color.r = 0.6;color.g = 0.2;color.b = 0.98;
    return color;
}


