#include "mars_slam/laserscan_processor.h"

LaserscanProcessor::LaserscanProcessor(ros::NodeHandle& nh_):nh(nh_),set_scan_corrected_tf(false)
{
    ROS_INFO_STREAM("Init LaserScan Processor !!");

    nh.param<double>("laser_min_range", laser_min_range, -1);
    nh.param<double>("laser_max_range", laser_max_range, -1); 
    nh.param<double>("voxel_size", voxel_size, 0.2); 
    nh.param<int>("max_laser_points", max_laser_points, 100); 

    scan_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("scan",1);
    scan_cloud_down_pub = nh.advertise<sensor_msgs::PointCloud2>("scan_down",1);
}

void LaserscanProcessor::setScanCorrectedTransform(const tf::Transform& transform_to_corrected_pose_)
{
    transform_to_corrected_pose = transform_to_corrected_pose_;
    tf::transformTFToEigen(transform_to_corrected_pose,tf_to_corrected_pose_eigen);
    set_scan_corrected_tf = true;
}

void LaserscanProcessor::processLaserscan(const sensor_msgs::LaserScanConstPtr& scan_msg,
                                                PCLScanCloudPtr& scan_cloud,
                                                PCLScanCloudPtr& scan_cloud_down)
{
    convertLaserScanToPCLPointCloud(scan_msg,scan_cloud);
    correctLaserScan(scan_cloud);
    filterPoints(scan_cloud,scan_cloud_down);
}

void LaserscanProcessor::convertLaserScanToPCLPointCloud(const sensor_msgs::LaserScanConstPtr& scan_msg,
                                                        PCLScanCloudPtr& scan_cloud)
{
    static bool first = true;

    if(first)
    {
        first = false;
        if(laser_min_range < 0)
            laser_min_range = scan_msg->range_min;
        if(laser_max_range < 0)
            laser_max_range = scan_msg->range_max;
    }

    scan_cloud->points.reserve(scan_msg->ranges.size());

    for(unsigned int i = 0; i < scan_msg->ranges.size();i++)
    {
        double lx,ly,angle;

        if(scan_msg->ranges[i] < scan_msg->range_min || 
           scan_msg->ranges[i] > scan_msg->range_max || 
           scan_msg->ranges[i] > laser_max_range     ||
           scan_msg->ranges[i] < laser_min_range)
            continue;
        else{
            angle = scan_msg->angle_min + scan_msg->angle_increment * i;
            lx = scan_msg->ranges[i] * std::cos(angle);
            ly = scan_msg->ranges[i] * std::sin(angle);
        }

        if(std::isnan(lx) || std::isinf(ly) || std::isnan(ly) || std::isinf(ly))
            continue;
        
        pcl::PointXYZ pt(lx,ly,0); 
        scan_cloud->points.push_back(pt);
    }

    scan_cloud->width = scan_cloud->points.size();
    scan_cloud->height = 1;
}

void LaserscanProcessor::correctLaserScan(PCLScanCloudPtr& scan_cloud)
{
    PCLScanCloudPtr scan_cloud_corrected (new PCLScanCloud);
    pcl::transformPointCloud (*scan_cloud, *scan_cloud_corrected, tf_to_corrected_pose_eigen);
    *scan_cloud = *scan_cloud_corrected;
}

void LaserscanProcessor::filterPoints(const PCLScanCloudPtr& cloud_in,
                                            PCLScanCloudPtr& cloud_out)
{   
    voxelFilter(cloud_in,cloud_out);
    downSample(cloud_out);
}

void LaserscanProcessor::voxelFilter(const PCLScanCloudPtr& cloud_in,
                                           PCLScanCloudPtr& cloud_out)
{
    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(cloud_in);
    voxel_filter.setLeafSize(voxel_size,voxel_size,voxel_size);
    voxel_filter.filter(*cloud_out);
}

void LaserscanProcessor::downSample(PCLScanCloudPtr& cloud)
{
    PCLScanCloudPtr cloud_down (new PCLScanCloud);

    cloud_down->points.reserve(cloud->points.size());

    int laser_skip = floor(cloud->points.size()/max_laser_points);

    for(unsigned int i = 0; i < cloud->points.size(); i++)
        if(i%(laser_skip+1) == 0)
            cloud_down->points.push_back(cloud->points[i]);

    cloud = cloud_down;
}

void LaserscanProcessor::publishPts(const PCLScanCloudPtr& cloud,
                                    const PCLScanCloudPtr& cloud_down,
                                    const tf::Transform& pose,
                                    const ros::Time& t)                
{
    PCLScanCloudPtr cloud_map(new PCLScanCloud);
    PCLScanCloudPtr cloud_down_map(new PCLScanCloud);

    transformPointsToMapFrame(cloud,cloud_map,pose);
    transformPointsToMapFrame(cloud_down,cloud_down_map,pose);

    sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_map, *cloud_msg);
    cloud_msg->header.frame_id = "map";
    cloud_msg->header.stamp = t;
    scan_cloud_pub.publish(cloud_msg);

    sensor_msgs::PointCloud2Ptr cloud_down_msg(new sensor_msgs::PointCloud2);
    pcl::toROSMsg(*cloud_down_map, *cloud_down_msg);
    cloud_down_msg->header.frame_id = "map";
    cloud_down_msg->header.stamp = t;
    scan_cloud_down_pub.publish(cloud_down_msg);
}

void LaserscanProcessor::transformPointsToMapFrame(const PCLScanCloudPtr& cloud_in,
                                                         PCLScanCloudPtr& cloud_out,
                                                   const tf::Transform& pose)
{
    Eigen::Affine3d pose_eigen;
    tf::transformTFToEigen(pose,pose_eigen);
    pcl::transformPointCloud(*cloud_in, *cloud_out,pose_eigen);
}
