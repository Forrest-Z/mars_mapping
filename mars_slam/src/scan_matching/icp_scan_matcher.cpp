#include "mars_slam/scan_matching/icp_scan_matcher.h"

IcpScanMatcher::IcpScanMatcher(ros::NodeHandle& nh_):
nh(nh_),
auto_update_count_down(40),
auto_update(false),
update_map_cloud(false)
{
    ROS_INFO_STREAM("Init ICP Scan Matcher !!");

    nh.param<double>("scan_matching/icp/submap_size", submap_size, 30.0);
    // nh.param<double>("scan_matching/icp/submap_voxel_size", submap_voxel_size, 0.05);
    nh.param<int>("scan_matching/icp/max_iteration", max_iteration, 100);
    nh.param<double>("scan_matching/icp/max_correspondence_distance", max_correspondence_distance, 0.05);
    nh.param<double>("scan_matching/icp/transformation_epsilon", transformation_epsilon, 1e-10);
    nh.param<double>("scan_matching/icp/euclidean_fitness_epsilon", euclidean_fitness_epsilon, 0.1);
    nh.param<bool>("scan_matching/icp/one_pixel_to_four_points", one_pixel_to_four_points, true);

    icp.setMaxCorrespondenceDistance (max_correspondence_distance); 
    icp.setMaximumIterations (max_iteration);
    icp.setTransformationEpsilon (transformation_epsilon);
    icp.setEuclideanFitnessEpsilon (euclidean_fitness_epsilon);

    PCLScanCloudPtr new_cloud_map(new PCLScanCloud);
    cloud_map = new_cloud_map;
    cloud_map_pub = nh.advertise<sensor_msgs::PointCloud2>("map_cloud",1,false);
}

tf::Pose IcpScanMatcher::icpLocalization(tf::Pose &init_pose,const PCLScanCloudPtr& cloud,
                                                             const PCLScanCloudPtr& cloud_down)
{
    static bool first = true;
    tf::Pose final_pose;
    PCLScanCloudPtr output_cloud(new PCLScanCloud);
    
    if(first)
    {
        first = false;
        final_pose = init_pose;
    }else{
        // scan matching process
        Eigen::Matrix4f guess = Eigen::Matrix4f::Identity();
        Eigen::Matrix4f final = Eigen::Matrix4f::Identity();

        tfPoseToEigenPose(init_pose,guess);
        // icp main process
        icp.setInputSource(cloud_down);
        icp.setInputTarget(cloud_map);
        icp.align(*output_cloud,guess);
        final = icp.getFinalTransformation();
        eigenPoseToTfPose(final,final_pose);   
    }


    if(!auto_update)
    {
        transformPoints(cloud,output_cloud,final_pose);
        updateMapCloudByNewCloud(output_cloud);
        update_map_cloud = true;
        auto_update_count_down--;
        if(auto_update_count_down == 0)
            auto_update = true;
    }

    return final_pose;
}

void IcpScanMatcher::tfPoseToEigenPose(const tf::Pose& tf_pose,Eigen::Matrix4f& eigen_pose)
{
    Eigen::Affine3d ad;
    Eigen::Affine3f af;
    tf::transformTFToEigen(tf_pose,ad);
    af = ad.cast<float>();
    eigen_pose = af.matrix();
}

void IcpScanMatcher::eigenPoseToTfPose(const Eigen::Matrix4f& eigen_pose,tf::Pose& tf_pose)
{
    Eigen::Affine3d ad;
    Eigen::Affine3f af;
    af = eigen_pose;
    ad = af.cast<double>();
    tf::transformEigenToTF(ad,tf_pose);
}

void IcpScanMatcher::transformPoints(const PCLScanCloudPtr& cloud_in,
                                           PCLScanCloudPtr& cloud_out,
                                     const tf::Transform& transform)
{
    PCLScanCloudPtr transformed_cloud(new PCLScanCloud);
    Eigen::Affine3d transform_eigen;
    tf::transformTFToEigen(transform,transform_eigen);
    pcl::transformPointCloud(*cloud_in, *transformed_cloud, transform_eigen);
    cloud_out = transformed_cloud;
}

// void IcpScanMatcher::filterCloudByVoxel(PCLScanCloudPtr& cloud)
// {
//     PCLScanCloudPtr filtered_cloud (new PCLScanCloud);
//     pcl::VoxelGrid<PCLScan> voxel_filter;

//     voxel_filter.setInputCloud (cloud);
//     voxel_filter.setLeafSize (submap_voxel_size,submap_voxel_size, submap_voxel_size);
//     voxel_filter.filter (*filtered_cloud);

//     cloud = filtered_cloud;
// }

void IcpScanMatcher::publishMapCloud()                
{
    if(update_map_cloud){
        sensor_msgs::PointCloud2Ptr cloud_msg(new sensor_msgs::PointCloud2);
        update_cloud_map_mutex.lock();
        pcl::toROSMsg(*cloud_map, *cloud_msg);
        update_cloud_map_mutex.unlock();
        cloud_msg->header.frame_id = "map";
        cloud_msg->header.stamp = ros::Time();
        cloud_map_pub.publish(cloud_msg);
    }
}

void IcpScanMatcher::updateMapCloudByNewCloud(const PCLScanCloudPtr& cloud)
{
    PCLScanCloudPtr new_cloud_map (new PCLScanCloud); 

    *new_cloud_map = *cloud_map + *cloud; 
    new_cloud_map->width = new_cloud_map->points.size();
    new_cloud_map->height = 1;
    update_cloud_map_mutex.lock();
    cloud_map = new_cloud_map;
    update_cloud_map_mutex.unlock();
}

void IcpScanMatcher::updateMapCloudFromOccMap(OccMap& occ_map,const tf::Pose& current_pose)
{
    if(!auto_update)
        return;

    ros::Time t1 = ros::Time::now();
    occ_map.lockMapUpdate(1);

    PCLScanCloudPtr new_cloud_map (new PCLScanCloud); 
    MapInfo map_info = occ_map.getMapInfo();

    int width = map_info.width;
    int height = map_info.height;
    int map_size = map_info.map_size;
    double resolution = map_info.resolution;
    double origin_x = map_info.origin.x;
    double origin_y = map_info.origin.y;
    unsigned char* log_map = occ_map.getMapData();
    double now_x = current_pose.getOrigin().x();
    double now_y = current_pose.getOrigin().y();
    // int log_confirm_occ = occ_map.confirmOccValue();
    int log_confirm_occ = 101;


    int start_x = floor((now_x - submap_size/2 - origin_x)/resolution);
    int end_x   = floor((now_x + submap_size/2 - origin_x)/resolution);
    int start_y = floor((now_y - submap_size/2 - origin_y)/resolution);
    int end_y   = floor((now_y + submap_size/2 - origin_y)/resolution);

    if(start_x < 0)start_x = 0;
    if(end_x >= width)end_x = width-1;
    if(start_y < 0)start_y = 0;
    if(end_y >= height)end_y = height-1;

    for(unsigned int i = start_y ;i <= end_y; i++)
    {
        for(unsigned int j = start_x ;j <= end_x; j++)
        {
            int index = j + i * width;
            if(log_map[index] >= log_confirm_occ)
            {
                double x = origin_x + (double)j*resolution + resolution/2;
                double y = origin_y + (double)i*resolution + resolution/2;

                if(one_pixel_to_four_points)
                {
                    double d = resolution/4;
                    pcl::PointXYZ p1(x+d,y+d,0.0);
                    pcl::PointXYZ p2(x+d,y-d,0.0);
                    pcl::PointXYZ p3(x-d,y+d,0.0);
                    pcl::PointXYZ p4(x-d,y-d,0.0);
                    new_cloud_map->points.push_back(p1);
                    new_cloud_map->points.push_back(p2);
                    new_cloud_map->points.push_back(p3);
                    new_cloud_map->points.push_back(p4);
                }else{
                    pcl::PointXYZ p(x,y,0.0);
                    new_cloud_map->points.push_back(p);
                }
            }
        }
    }

    new_cloud_map->width = new_cloud_map->points.size();
    new_cloud_map->height = 1;

    update_cloud_map_mutex.lock();
    cloud_map = new_cloud_map;
    update_cloud_map_mutex.unlock();
    
    occ_map.unlockMapUpdate(1);
    ros::Time t2 = ros::Time::now();
    ROS_DEBUG_STREAM("Time for update map cloud : " <<t2 - t1);
}