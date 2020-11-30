#include "mars_db_comparator/map_manager.h"

MapManager::MapManager(ros::NodeHandle& nh_):nh(nh_)
{
    nh.param<std::string>("db_file_path", db_file_path, "");
    nh.param<double>("scan_map_resolution",scan_map_resolution,0.05);

    loadMap(db_file_path);

    scan_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("scan_map",1,true);
    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("current_map",1,true);
}

void MapManager::loadMap(const std::string& file_path)
{
    std::ifstream file(file_path);

    if(!file.is_open())
    {
        ROS_ERROR_STREAM("Fail to open file !!");
        return;
    }else{
        ROS_INFO_STREAM("Open "<< file_path);
    }

    getSstream(file) >> submap_num;
    getSstream(file) >> resolution;

    submaps.clear();
    submaps.reserve(submap_num);

    for(unsigned int i = 0;i < submap_num;i++)
        submaps.push_back(readMap(file));

    file.close();
    ROS_INFO_STREAM("Success to read maps from db file !!");
}

std::stringstream MapManager::getSstream(std::ifstream& file)
{
    std::string inputLine; 
    std::stringstream ss;

    getline(file,inputLine);
    ss.str(inputLine);

    return ss;
}

nav_msgs::OccupancyGridPtr MapManager::readMap(std::ifstream& file)
{
    nav_msgs::OccupancyGridPtr new_map(new nav_msgs::OccupancyGrid);

    new_map->header.frame_id = "map";
    new_map->header.stamp = ros::Time();
    new_map->info.resolution = resolution;
    getSstream(file) >> new_map->info.width >> new_map->info.height;
    getSstream(file) >> new_map->info.origin.position.x 
                     >> new_map->info.origin.position.y
                     >> new_map->info.origin.position.z
                     >> new_map->info.origin.orientation.x 
                     >> new_map->info.origin.orientation.y
                     >> new_map->info.origin.orientation.z
                     >> new_map->info.origin.orientation.w;   
    int map_size = new_map->info.width * new_map->info.height;
    new_map->data.reserve(map_size);
    std::string inputLine; 
    getline(file,inputLine);

    for(unsigned int i = 0;i < map_size; i++){
        unsigned char temp = inputLine[i];
        int temp_ = temp;
        temp_ = (temp_ - 40)/2;
        if(temp_ > 100)
            temp_ = 100;
        if(temp_ < 0)
            temp_ = 0;
        new_map->data.push_back(temp_);
    }

    return new_map;
}

void MapManager::transferPointCloudToOccMap(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                                  nav_msgs::OccupancyGridPtr& map)
{
    nav_msgs::OccupancyGridPtr map_new(new nav_msgs::OccupancyGrid);
    map = map_new;
    setMapInfo(cloud,map);
    setMapData(cloud,map);
    scan_map_pub.publish(map);
}

void MapManager::setMapInfo(const pcl::PointCloud<pcl::PointXYZ>::Ptr& cloud,
                                   nav_msgs::OccupancyGridPtr& map)
{
    MaxMin max_min = findCloudMaxMin(cloud);    

    map->header.frame_id = "map";
    map->header.stamp = ros::Time();
    map->info.resolution = scan_map_resolution;
    tf::poseTFToMsg(tf::Pose::getIdentity(),map->info.origin);
    map->info.origin.position.x = max_min.min_x - map->info.resolution;
    map->info.origin.position.y = max_min.min_y - map->info.resolution;  
    map->info.width  = ceil((max_min.max_x - max_min.min_x)/map->info.resolution) + 2;
    map->info.height = ceil((max_min.max_y - max_min.min_y)/map->info.resolution) + 2;
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
        int x = floor((pt.x - origin_x)/map->info.resolution);
        int y = floor((pt.y - origin_y)/map->info.resolution);
        int index = x + y * w;
        map->data[index] = 100;
    }
}

void MapManager::pubMap(const int& n)
{
    map_pub.publish(submaps[n]);
}