#ifndef MAP_H
#define MAP_H

#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <tf/transform_datatypes.h>
#include "mars_slam/scan.h"

typedef struct 
{
    double resolution;
    int width;
    int height;
    int map_size;
    geometry_msgs::Point origin;

    void set(double resolution_,int width_,int height_,geometry_msgs::Point origin_)
    {
        resolution = resolution_;
        width = width_;
        height = height_;
        map_size = width * height;
        origin = origin_;
    }

}MapInfo;

typedef struct
{
    PCLScanCloudPtr cloud;
    PCLScanCloudPtr cloud_down;
    tf::Pose pose;
    ros::Time time;

}CellsData;

typedef struct
{
    PCLScanCloudPtr cloud;
    tf::Pose pose;
    int current_submap_serial_num;
    int scan_serial_num;
    
}InsertedCellsData;

typedef struct
{
    double x;
    double y;
    int score;

}TransformedGrid;



typedef std::shared_ptr<CellsData> CellsDataPtr;

#endif 