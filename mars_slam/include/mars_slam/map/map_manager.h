#ifndef MAP_MANAGER_H
#define MAP_MANAGER_H

#include <future>
#include <omp.h>
#include <vector>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "mars_slam/map/occ_map.h"
#include "mars_slam/map/map_info_viewer.h"
#include "mars_slam/loop_closure_detection/map_stacks.h"
#include "mars_slam/optimization/pose_optimizer.h"

class MapManager
{
public:
    explicit MapManager(){}
    explicit MapManager(std::shared_ptr<ros::NodeHandle>& nh_,std::shared_ptr<PoseOptimizer>& pose_optimizer_);
    // MapManager(const MapManager&) = delete;
    ~MapManager(){}
    void dealWithNewScanData(const PCLScanCloudPtr& scan_cloud,const tf::Pose& pose,
                                   int& current_submap_serial_num,int& scan_serial_num);
    void deleteMap();
    std::shared_ptr<OccMap>& getScanMatchingMap();
    std::shared_ptr<MapStacks>& mapStacks(){return map_stacks;}
    void updateSubmapByOptimizationPoses(const int& last_submap_serial_num);
    bool callSaveMap(const std::string& file_name);
    void stopPubGlobalMap(){stop_pub_global_map = true;}

private:
    void publishMapLoop();
    bool checkChangeSubmap(const tf::Pose& pose);
    double dist(const tf::Pose& a,const tf::Pose& b);
    void publishMap();
    void publishGlobalMap();
    void addNewSubmap(const tf::Pose& pose);
    void newSubmapProcess(const tf::Pose& pose);
    bool saveMap(const std::string& file_path);
    void saveMapInfo(std::ofstream &file);

    std::shared_ptr<ros::NodeHandle> nh;
    std::unique_ptr<MapInfoViewer> map_info_viewer;

    // param
    int initial_map_size,omp_max_thread_num;
    double map_resolution,change_submap_dis,map_publsih_frequency;
    bool pub_submap;

    // threads
    std::future<void> map_pub_thread;
    boost::mutex update_global_map_mutex;

    // submaps
    std::vector<std::shared_ptr<OccMap>> submaps;
    std::shared_ptr<OccMap> first_submap;
    std::shared_ptr<OccMap> second_submap;

    // global map
    std::shared_ptr<OccMap> global_map;
    std::shared_ptr<OccMap> middle_map;
    boost::mutex middle_map_mutex;
    bool stop_pub_global_map;

    // map stacks for loop closure detection
    std::shared_ptr<MapStacks> map_stacks;

    // pose optimizer
    std::shared_ptr<PoseOptimizer> pose_optimizer;

};

#endif 