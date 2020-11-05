#ifndef SCAN_MATCHER_H
#define SCAN_MATCHER_H

#include <future>
#include <ros/ros.h>
#include "mars_slam/scan_matching/icp_scan_matcher.h"
#include "mars_slam/map/map_manager.h"
class ScanMatcher
{
public:
    explicit ScanMatcher(){}
    explicit ScanMatcher(std::shared_ptr<ros::NodeHandle>& nh_,std::shared_ptr<MapManager>& map_manager_);
    ScanMatcher(const ScanMatcher&) = delete;
    ~ScanMatcher(){}
    tf::Pose doScanMatching(tf::Pose &init_pose, const PCLScanCloudPtr& cloud,
                                                 const PCLScanCloudPtr& cloud_down);
    void callStopUpdateCloud(){stop_update = true;}
    bool successToStop(){return success_to_stop;}

private:
    void updateMapCloudLoop();

    std::shared_ptr<ros::NodeHandle> nh;
    std::unique_ptr<IcpScanMatcher> icp_scan_matcher;

    double update_map_cloud_frequency;

    // threads
    std::future<void> cloud_map_update_thread;

    // map manager
    std::shared_ptr<MapManager> map_manager;

    tf::Pose current_pose;

    // stop update
    bool stop_update;
    bool success_to_stop;
};

#endif 