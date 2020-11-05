#ifndef MAP_STACKS_H
#define MAP_STACKS_H

#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include "mars_slam/debug.h"
#include "mars_slam/map/occ_map.h"

typedef std::vector<nav_msgs::OccupancyGridPtr> OccGridStack;
typedef std::shared_ptr<std::vector<nav_msgs::OccupancyGridPtr>> OccGridStackPtr;

class MapStacks
{
public:
    explicit MapStacks(){}
    explicit MapStacks(ros::NodeHandle& nh_);
    MapStacks(const MapStacks&) = delete;
    ~MapStacks(){}
    void updateMapStack(OccMap& map);
    const int& numOfStacks(){return num_of_stacks;}
    const tf::Pose& getCenter(const int& n){return submap_centers_array[n];}
    const tf::Pose& getTransformedCenter(const int& n){return submap_transformed_centers_array[n];}
    void setTransformedCenter(const int& n,const tf::Pose& pose){submap_transformed_centers_array[n] = pose;}
    OccGridStackPtr& getStack(const int& n){return (*occ_map_stacks)[n];}

private:
    void addNewStack(OccGridStackPtr& new_stack,tf::Pose& center);
    // void renewStack(const int& n,OccGridStackPtr& new_stack,tf::Pose& center);
    void addNewStackPublishers();
    void setMap(nav_msgs::OccupancyGridPtr& new_map,OccMap& origin_map);
    void inflateMap(nav_msgs::OccupancyGridPtr& map);
    OccGridStackPtr produceNewStack(const nav_msgs::OccupancyGridPtr& map,const MapInfo& origin_map_info);
    nav_msgs::OccupancyGridPtr initPrecomputedGrid(
            const int& depth,const nav_msgs::OccupancyGridPtr& map,const MapInfo& origin_map_info);
    void pubStack(const int& serial_num);

    ros::NodeHandle nh;

    int num_of_stacks;

    // param
    bool pub_grid_stack,inflate_map;
    int branch_and_bound_depth,inflate_pixels;

    std::shared_ptr<std::vector<OccGridStackPtr>> occ_map_stacks;
    std::vector<tf::Pose> submap_centers_array;
    std::vector<tf::Pose> submap_transformed_centers_array;
    // std::vector<MapInfo> origin_map_info_array;

    // publishers
    std::vector<std::vector<ros::Publisher>> stacks_pubisher_array;

    // mutex
    std::vector<std::shared_ptr<boost::mutex>> stack_mutex_array;

};

#endif 