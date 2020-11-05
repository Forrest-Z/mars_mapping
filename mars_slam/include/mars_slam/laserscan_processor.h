#ifndef LASERSCAN_PROCESSOR_H
#define LASERSCAN_PROCESSOR_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "mars_slam/scan.h"

class LaserscanProcessor
{
public:
    explicit LaserscanProcessor(){}
    explicit LaserscanProcessor(ros::NodeHandle& nh_);
    ~LaserscanProcessor(){}
    LaserscanProcessor(const LaserscanProcessor&) = delete;
    bool setScanCorrectedTF(){return set_scan_corrected_tf;}
    void setScanCorrectedTransform(const tf::Transform& transform_to_corrected_pose_);
    void processLaserscan(const sensor_msgs::LaserScanConstPtr& scan_msg,
                                PCLScanCloudPtr& scan_cloud,
                                PCLScanCloudPtr& scan_cloud_down);
    void publishPts(const PCLScanCloudPtr& cloud,
                    const PCLScanCloudPtr& cloud_down,
                    const tf::Transform& pose,
                     const ros::Time& t);
                                        
private:
    void convertLaserScanToPCLPointCloud(const sensor_msgs::LaserScanConstPtr& scan_msg,
                                               PCLScanCloudPtr& scan_cloud);
    void correctLaserScan(PCLScanCloudPtr& scan_cloud); 
    void filterPoints(const PCLScanCloudPtr& cloud_in,
                            PCLScanCloudPtr& cloud_out);
    void voxelFilter(const PCLScanCloudPtr& cloud_in,
                           PCLScanCloudPtr& cloud_out);
    void downSample(PCLScanCloudPtr& cloud);
    void transformPointsToMapFrame(const PCLScanCloudPtr& cloud_in,
                                         PCLScanCloudPtr& cloud_out,
                                   const tf::Transform& pose);
                                               
    ros::NodeHandle nh;

    bool set_scan_corrected_tf;
    double laser_min_range,laser_max_range,voxel_size;
    int max_laser_points;

    tf::Transform transform_to_corrected_pose;
    Eigen::Affine3d tf_to_corrected_pose_eigen;

    ros::Publisher scan_cloud_pub;
    ros::Publisher scan_cloud_down_pub;
};


#endif