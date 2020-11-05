#ifndef ICP_SCAN_MATCHER_H
#define ICP_SCAN_MATCHER_H

#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <pcl/registration/icp.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include "mars_slam/scan.h"
#include "mars_slam/map/occ_map.h"

class IcpScanMatcher
{
public:
    explicit IcpScanMatcher(){}
    explicit IcpScanMatcher(ros::NodeHandle& nh_);
    IcpScanMatcher(const IcpScanMatcher&) = delete;
    ~IcpScanMatcher(){}
    tf::Pose icpLocalization(tf::Pose &init_pose, const PCLScanCloudPtr& cloud,
                                                  const PCLScanCloudPtr& cloud_down);
    void publishMapCloud();
    void updateMapCloudFromOccMap(OccMap& occ_map,const tf::Pose& current_pose);

private:
    void eigenPoseToTfPose(const Eigen::Matrix4f& eigen_pose,tf::Pose& tf_pose);
    void tfPoseToEigenPose(const tf::Pose& tf_pose,Eigen::Matrix4f& eigen_pose);
    void transformPoints(const PCLScanCloudPtr& cloud_in,
                                               PCLScanCloudPtr& cloud_out,
                                         const tf::Transform& transform);
    // void filterCloudByVoxel(PCLScanCloudPtr& cloud);
    void updateMapCloudByNewCloud(const PCLScanCloudPtr& cloud);

    ros::NodeHandle nh;
    ros::Publisher cloud_map_pub;
    pcl::IterativeClosestPoint<PCLScan, PCLScan> icp;

    int max_iteration;
    double submap_size,submap_voxel_size;
    double max_correspondence_distance,transformation_epsilon,euclidean_fitness_epsilon;
    bool one_pixel_to_four_points;

    int auto_update_count_down;
    bool auto_update,update_map_cloud;

    PCLScanCloudPtr cloud_map;

    boost::mutex update_cloud_map_mutex;

};

#endif 