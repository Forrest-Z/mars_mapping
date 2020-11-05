#include "scan_line_detector/laserscan_processor.h"

LaserscanProcessor::LaserscanProcessor(ros::NodeHandle& nh_,const sensor_msgs::LaserScanConstPtr& msg):
nh(nh_),get_laser_transform(false)
{
    nh.param<double>("laser_min_range", laser_min_range, -1);
    nh.param<double>("laser_max_range", laser_max_range, -1); 
    nh.param<double>("voxel_size", voxel_size, 0.2); 
    nh.param<int>("max_laser_points", max_laser_points, 100); 

    if(laser_min_range < 0)
        laser_min_range = msg->range_min;
    if(laser_max_range < 0)
        laser_max_range = msg->range_max;

    nh.param<std::string>("robot_frame_id", robot_frame_id, "base_footprint"); 
    laser_frame_id = msg->header.frame_id;

    while(ros::ok() && !get_laser_transform)
    {
        if(getTransformToCorrectedPose()){
            get_laser_transform = true;
            ROS_INFO_STREAM("Success to get laserscan transform !!");
            break;
        }    
        ros::Duration(0.2).sleep();
    }
}

bool LaserscanProcessor::getStaticTransform(tf::Transform& transform,
                                      const std::string& base_frame, const std::string& target_frame)                
{
    tf::StampedTransform transform_;
    try{
        tf_listener.lookupTransform(base_frame, target_frame, ros::Time(), transform_);                         
    }
    catch (tf::TransformException ex){
        ROS_WARN_STREAM("Failed to compute transform from "<< 
                        base_frame << " to " << target_frame << " , " <<ex.what());
        return false;
    }    
    transform = transform_;
    return true;
}

// get transform from laser to robot plane
bool LaserscanProcessor::getTransformToCorrectedPose()
{
    tf::Transform laser_on_robot,corrected_laser_on_robot;

    if(!getStaticTransform(laser_on_robot,robot_frame_id,laser_frame_id))
        return false;
    
    corrected_laser_on_robot = laser_on_robot;
    tf::Quaternion q;
    double yaw = tf::getYaw(corrected_laser_on_robot.getRotation());
    q.setRPY(0.0,0.0,yaw);
    corrected_laser_on_robot.setRotation(q);
    transform_to_corrected_pose = corrected_laser_on_robot.inverse()*laser_on_robot;
    tf::transformTFToEigen(transform_to_corrected_pose,tf_to_corrected_pose_eigen);

    return true;
}

void LaserscanProcessor::processLaserscan(const sensor_msgs::LaserScanConstPtr& scan_msg,
                                            pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud)
{
    convertLaserScanToPCLPointCloud(scan_msg,scan_cloud);
    correctLaserScan(scan_cloud);
    filterPoints(scan_cloud);
}

void LaserscanProcessor::convertLaserScanToPCLPointCloud(const sensor_msgs::LaserScanConstPtr& scan_msg,
                                                        pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud)
{
    scan_cloud->points.clear();
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

void LaserscanProcessor::correctLaserScan(pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud_corrected (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::transformPointCloud (*scan_cloud, *scan_cloud_corrected, tf_to_corrected_pose_eigen);
    scan_cloud->points.clear();
    *scan_cloud = *scan_cloud_corrected;
}

void LaserscanProcessor::filterPoints(pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud)
{
    voxelFilter(scan_cloud);
    downSample(scan_cloud);
}

void LaserscanProcessor::voxelFilter(pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_voxel (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> voxel_filter;
    voxel_filter.setInputCloud(scan_cloud);
    voxel_filter.setLeafSize(voxel_size,voxel_size,voxel_size);
    voxel_filter.filter(*scan_voxel);

    scan_cloud = scan_voxel;
}

void LaserscanProcessor::downSample(pcl::PointCloud<pcl::PointXYZ>::Ptr& scan_cloud)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_downsample (new pcl::PointCloud<pcl::PointXYZ>);

    scan_downsample->points.clear();
    scan_downsample->points.reserve(scan_cloud->points.size());

    int laser_skip = floor(scan_cloud->points.size()/max_laser_points);

    for(unsigned int i = 0; i < scan_cloud->points.size(); i++)
        if(i%(laser_skip+1) == 0)
            scan_downsample->points.push_back(scan_cloud->points[i]);

    scan_cloud = scan_downsample;
}