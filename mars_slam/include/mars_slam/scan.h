#ifndef SCAN_H
#define SCAN_H   

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

typedef std::vector<Eigen::Vector2d> LaserScan;
typedef std::shared_ptr<LaserScan> LaserScanPtr;
typedef pcl::PointXYZ PCLScan;
typedef pcl::PointCloud<pcl::PointXYZ> PCLScanCloud;
typedef pcl::PointCloud<pcl::PointXYZ>::Ptr PCLScanCloudPtr;

#endif 
