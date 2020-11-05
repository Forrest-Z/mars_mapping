#include "mars_slam/scan_matching/scan_matcher.h"

ScanMatcher::ScanMatcher(std::shared_ptr<ros::NodeHandle>& nh_,std::shared_ptr<MapManager>& map_manager_)
:nh(nh_),
map_manager(map_manager_),
stop_update(false),
success_to_stop(false)
{
    icp_scan_matcher = std::make_unique<IcpScanMatcher>(*nh);

    current_pose = tf::Pose::getIdentity();

    nh->param<double>("scan_matching/icp/update_map_cloud_frequency", update_map_cloud_frequency, 10);

    // open threads
    cloud_map_update_thread = std::async(std::launch::async, &ScanMatcher::updateMapCloudLoop,this);
    ROS_INFO_STREAM("Open map cloud thread, start publish map cloud !!");
    ROS_INFO_STREAM(" ");
}

tf::Pose ScanMatcher::doScanMatching(tf::Pose &init_pose, const PCLScanCloudPtr& cloud,
                                                           const PCLScanCloudPtr& cloud_down)
{
    ros::Time t1 = ros::Time::now();
    tf::Pose final_pose;
    final_pose =  icp_scan_matcher->icpLocalization(init_pose,cloud,cloud_down);
    current_pose = final_pose;
    ros::Time t2 = ros::Time::now();
    ROS_DEBUG_STREAM("Time for icp scan matching: " << t2 -t1);
    return final_pose;
}

void ScanMatcher::updateMapCloudLoop()
{
    ros::Rate r(update_map_cloud_frequency); 

    while(ros::ok() && !stop_update)
    {
        std::shared_ptr<OccMap> scan_matching_map = map_manager->getScanMatchingMap(); 
        icp_scan_matcher->updateMapCloudFromOccMap(*scan_matching_map,current_pose);
        icp_scan_matcher->publishMapCloud();
        ROS_DEBUG_STREAM("Pub map cloud !!");
        r.sleep();
    }
    ROS_INFO_STREAM("Stop updating map cloud");
    success_to_stop = true;
}