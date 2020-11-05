#include "scan_line_detector/map_manager.h"

MapManager::MapManager(ros::NodeHandle& nh_):nh(nh_)
{
    nh.param<double>("resolution", resolution, 0.05);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("scan_map",1,true);
}

void MapManager::transferPointCloudToOccMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                  nav_msgs::OccupancyGridPtr& map)
{
    nav_msgs::OccupancyGridPtr map_new(new nav_msgs::OccupancyGrid);
    map = map_new;
    setMapInfo(cloud,map);
    setMapData(cloud,map);
    map_pub.publish(map);
}

void MapManager::setMapInfo(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                   nav_msgs::OccupancyGridPtr& map)
{
    MaxMin max_min = findCloudMaxMin(cloud);    

    map->header.frame_id = "map";
    map->header.stamp = ros::Time();
    map->info.resolution = resolution;
    tf::poseTFToMsg(tf::Pose::getIdentity(),map->info.origin);
    map->info.origin.position.x = max_min.min_x - resolution;
    map->info.origin.position.y = max_min.min_y - resolution;  
    map->info.width  = ceil((max_min.max_x - max_min.min_x)/resolution) + 2;
    map->info.height = ceil((max_min.max_y - max_min.min_y)/resolution) + 2;
}

MaxMin MapManager::findCloudMaxMin(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud)
{
    MaxMin max_min;
    double max_x,max_y,min_x,min_y;


    max_x = min_x = cloud->points[0].x;
    max_y = min_y = cloud->points[0].y;

    for(unsigned int i = 1;i < cloud->points.size();i++)
    {
        pcl::PointXYZ pt = cloud->points[i];

        if(pt.x > max_x)
            max_x = pt.x;
        if(pt.y > max_y)
            max_y = pt.y;
        if(pt.x < min_x)
            min_x = pt.x;
        if(pt.y < min_y)
            min_y = pt.y;
    }

    max_min.max_x = max_x;
    max_min.max_y = max_y;
    max_min.min_x = min_x;
    max_min.min_y = min_y;

    return max_min;
}   


void MapManager::setMapData(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                  nav_msgs::OccupancyGridPtr& map)
{
    double origin_x = map->info.origin.position.x;
    double origin_y = map->info.origin.position.y;
    int w = map->info.width;
    int h = map->info.height;

    map->data.resize(map->info.width * map->info.height);
    

    for(unsigned int i = 0;i < cloud->points.size();i++)
    {
        pcl::PointXYZ pt = cloud->points[i];
        int x = floor((pt.x - origin_x)/resolution);
        int y = floor((pt.y - origin_y)/resolution);
        int index = x + y * w;
        map->data[index] = 100;
    }
}