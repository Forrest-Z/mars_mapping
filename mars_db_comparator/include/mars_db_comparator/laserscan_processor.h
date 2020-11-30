#ifndef LASERSCAN_PROCESSOR_H
#define LASERSCAN_PROCESSOR_H

#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>

class LaserscanProcessor
{
public:
    explicit LaserscanProcessor(ros::NodeHandle& nh_,const sensor_msgs::LaserScanConstPtr& msg);
    ~LaserscanProcessor(){}
    LaserscanProcessor(const LaserscanProcessor&) = delete;
    bool getStaticTransform(tf::Transform& transform,const std::string& base_frame, const std::string& target_frame);
    bool getTransformToCorrectedPose();
    void processLaserscan(const sensor_msgs::LaserScan& scan_msg,
                                pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud);
    void convertLaserScanToPCLPointCloud(const sensor_msgs::LaserScan& scan_msg,
                                         pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud);
    void correctLaserScan(pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud);                                            
    void filterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud);
    void voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud);
    void downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud);

private:
    ros::NodeHandle nh;

    double laser_min_range,laser_max_range,voxel_size;
    int max_laser_points;
    std::string robot_frame_id,laser_frame_id;

    tf::TransformListener tf_listener;
    bool get_laser_transform;
    tf::Transform transform_to_corrected_pose;
    Eigen::Affine3d tf_to_corrected_pose_eigen;
    

};

#endif 