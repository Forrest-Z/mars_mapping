#include "mars_db_comparator/map_manager.h"

MapManager::MapManager(ros::NodeHandle& nh_):nh(nh_)
{
    nh.param<std::string>("db_file_path", db_file_path, "");

    loadMap(db_file_path);
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