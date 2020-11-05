#include "mars_slam/loop_closure_detection/map_stacks.h"

MapStacks::MapStacks(ros::NodeHandle& nh_):nh(nh_),num_of_stacks(0)
{
    nh.param<bool>("loop_closure/pub_grid_stack", pub_grid_stack, true);
    nh.param<bool>("loop_closure/inflate_map", inflate_map, true);
    nh.param<int>("loop_closure/inflate_pixels", inflate_pixels, 1);
    nh.param<int>("loop_closure/branch_and_bound_depth", branch_and_bound_depth, 7);

    occ_map_stacks = std::make_shared<std::vector<OccGridStackPtr>>();
}

void MapStacks::updateMapStack(OccMap& map)
{
    int serial_num = map.getSerialNum();
    tf::Pose center = map.getCenter();
    ROS_DEBUG_STREAM(BOLDWHITE<<"Update map stack "<< serial_num);
    // MapInfo origin_map_info = map.getMapInfo();

    nav_msgs::OccupancyGridPtr new_map(new nav_msgs::OccupancyGrid);
    setMap(new_map,map);
    auto new_stack = produceNewStack(new_map,map.getMapInfo());

    if(serial_num >= num_of_stacks)
        addNewStack(new_stack,center);
    // else
    //     renewStack(serial_num,new_stack,center);
    
    if(pub_grid_stack)
        pubStack(serial_num);
}

void MapStacks::addNewStack(OccGridStackPtr& new_stack,tf::Pose& center)
{
    auto new_mutex = std::make_shared<boost::mutex>();
    stack_mutex_array.push_back(new_mutex);
    submap_centers_array.push_back(center);
    submap_transformed_centers_array.push_back(center);
    occ_map_stacks->push_back(new_stack);
    num_of_stacks = occ_map_stacks->size();

    if(pub_grid_stack)
        addNewStackPublishers();
}

// void MapStacks::renewStack(const int& n,OccGridStackPtr& new_stack,tf::Pose& center)
// {
//     stack_mutex_array[n]->lock();
//     (*occ_map_stacks)[n] = new_stack; 
//     submap_centers_array[n] = center;
//     stack_mutex_array[n]->unlock();
// }

void MapStacks::addNewStackPublishers()
{
    std::vector<ros::Publisher> new_publishers;
    new_publishers.resize(branch_and_bound_depth);
    int serial_num = num_of_stacks - 1;

    for(unsigned int i = 0;i < branch_and_bound_depth;i++)
    {
        new_publishers[i] = nh.advertise<nav_msgs::OccupancyGrid>
                        ("gridmap" + std::to_string(serial_num) + "_" + std::to_string(i),1,true);
    }

    stacks_pubisher_array.push_back(new_publishers);
}

void MapStacks::setMap(nav_msgs::OccupancyGridPtr& new_map,OccMap& origin_map)
{
    MapInfo map_info = origin_map.getMapInfo();
    int offset = (1 << (branch_and_bound_depth - 1)) - 1;
    int confirm_occ_value = origin_map.confirmOccValue();
    int confirm_free_value = origin_map.confirmFreeValue();
    unsigned char* map_data = origin_map.getMapData();

    new_map->header.frame_id = "map";
    new_map->header.stamp = ros::Time();
    new_map->info.resolution = map_info.resolution;
    new_map->info.width = map_info.width + offset;
    new_map->info.height = map_info.height + offset;
    new_map->info.origin.position = map_info.origin;
    int map_size = new_map->info.width * new_map->info.height;

    new_map->data.resize(map_size);

    for(unsigned int i = 0;i < new_map->info.height;i++)
    {
        for(unsigned int j = 0;j < new_map->info.width;j++)
        {
            int index_new = j + new_map->info.width * i;

            if(i < map_info.height && j < map_info.width)
            {
                int index_old = j + map_info.width * i;
                if(map_data[index_old] > confirm_occ_value)
                    new_map->data[index_new] = 100;
                else if(map_data[index_old] < confirm_free_value)
                    new_map->data[index_new] = 0;
                else
                    new_map->data[index_new] = 50;
            }else{
                new_map->data[index_new] = 50;
            }
        }
    }

    if(inflate_map)
        inflateMap(new_map);
}

void MapStacks::inflateMap(nav_msgs::OccupancyGridPtr& map)
{
    nav_msgs::OccupancyGridPtr temp_map(new nav_msgs::OccupancyGrid);
    *temp_map = *map;

    for(unsigned int i = inflate_pixels;i < map->info.height - inflate_pixels;i++)
    {
        for(unsigned int j = inflate_pixels;j < map->info.width - inflate_pixels;j++)
        {
            int index_new = i * map->info.width + j;
            int max = 0;

            for(int k = -inflate_pixels;k <= inflate_pixels;k++)
            {
                for(int d = -inflate_pixels;d <= inflate_pixels;d++)
                {
                    int index_old = (i+k) * map->info.width + (j+d);
                    if(map->data[index_old] > max)
                        max = map->data[index_old];
                }
            }
            temp_map->data[index_new] = max;
        }
    }
    map = temp_map;
}

OccGridStackPtr MapStacks::produceNewStack(const nav_msgs::OccupancyGridPtr& map,const MapInfo& origin_map_info)
{
    auto new_stack = std::make_shared<OccGridStack>();
    new_stack->reserve(branch_and_bound_depth);

    for(unsigned int i = 0;i < branch_and_bound_depth;i++)
        new_stack->push_back(initPrecomputedGrid(i,map,origin_map_info));

    return new_stack;
}

nav_msgs::OccupancyGridPtr MapStacks::initPrecomputedGrid(
        const int& depth,const nav_msgs::OccupancyGridPtr& map,const MapInfo& origin_map_info)
{
    nav_msgs::OccupancyGridPtr grid_map(new nav_msgs::OccupancyGrid);
    nav_msgs::OccupancyGridPtr temp_map(new nav_msgs::OccupancyGrid);

    *temp_map = *map;

    grid_map->header = map->header;
    grid_map->info.origin.position = origin_map_info.origin; 
    grid_map->info.resolution = origin_map_info.resolution;
    int w = grid_map->info.width = origin_map_info.width;
    int h = grid_map->info.height = origin_map_info.height;
    grid_map->data.resize(w*h);
    
    int window_size = 1 << depth;

    for(unsigned int i = 0;i < h;i++)
    {
        for(unsigned int j = 0;j < w;j++)
        {
            int index = j + map->info.width * i;
            int max_score = 0;
            for(unsigned int k = 0;k < window_size;k++)
            {
                int index_old = (j + k) + (map->info.width) * i;
                if(map->data[index_old] > max_score)
                    max_score = map->data[index_old];
            }
            temp_map->data[index] = max_score;
        } 
    } 

    for(unsigned int i = 0;i < w;i++)
    {
        for(unsigned int j = 0;j < h;j++)
        {
            int index = i + w * j;
            int max_score = 0;
            for(unsigned int k = 0;k < window_size;k++)
            {
                int index_old = i + (map->info.width) * (j+k);
                if(temp_map->data[index_old] > max_score)
                    max_score = temp_map->data[index_old];
            }
            grid_map->data[index] = max_score;
        } 
    }     

    return grid_map;
}

void MapStacks::pubStack(const int& serial_num)
{
    for(int i = 0;i < branch_and_bound_depth;i++)
    {
        nav_msgs::OccupancyGridPtr map = (*((*occ_map_stacks)[serial_num]))[i];
        stacks_pubisher_array[serial_num][i].publish(map);
    }
}

