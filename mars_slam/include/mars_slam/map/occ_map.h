#ifndef OCC_MAP_H
#define OCC_MAP_H 

#include <iostream>
#include <fstream> 
#include <omp.h>
#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>
#include <tf_conversions/tf_eigen.h>
#include <nav_msgs/OccupancyGrid.h>
#include "mars_slam/scan.h"
#include "mars_slam/map/map.h"
#include "mars_slam/optimization/pose_optimizer.h"
#include "mars_srvs/SaveMap.h"

class OccMap
{
public:
    explicit OccMap(){}
    explicit OccMap(const int& initial_map_size,
                    const double& resolution_,
                    const tf::Pose& center_,
                    const int& serial_num_);
    OccMap(const OccMap&) = delete;
    ~OccMap(){}
    void initMap(const int& initial_map_size,
                 const double& resolution_,
                 const tf::Pose& center_,
                 const int& serial_num_,
                 bool new_ = true);
    const tf::Pose& getCenter(){return transformed_center;} 
    void addNewScanData(const PCLScanCloudPtr& scan_cloud,const tf::Pose& robot_pose);
    void deleteMap();
    void initMapPub(ros::NodeHandle& nh);
    void publishMap();
    MapInfo& getMapInfo(){return *map_info;}
    void lockMapUpdate(int i){update_mutex[i].lock();}
    void unlockMapUpdate(int i){update_mutex[i].unlock();}
    unsigned char* getMapData(){return log_map;} 
    int confirmOccValue(){return log_confirm_occ;}
    int confirmFreeValue(){return log_confirm_free;}
    void setOwnScanNum(){own_scan_num = scan_data.size();}
    void combineMap(OccMap& m_);
    void copyMap(OccMap& m_);
    const int getSerialNum(){return serial_num;}
    const int getCurrentScanNum(){return scan_data.size();}
    void updateMapByOptimizationPoses(const VerticesPtr& first_poses_array,const VerticesPtr& second_poses_array);
    void copyOriginalMapToHalfMap();
    void createTransformedMap();
    void combineMapByTransformedMap(OccMap& m_);
    void narrowMap();
    void saveMap(std::ofstream &file);

private:
    void updateMinMax();
    Eigen::Matrix3d toMatrix(const tf::Pose& robot_pose);
    Eigen::Vector2d transPointToGlobalFrame(const Eigen::Vector2d& pt,const Eigen::Matrix3d& T);
    void checkMapSize();
    void expandMap(const double& min_x_new,const double& min_y_new,
                   const double& max_x_new,const double& max_y_new);  
    void traceLine(int x0, int y0, int x1, int y1,
                       std::vector<Eigen::Vector2i>& grid_index_vector); 
    void setMapInfo();
    void renewPoses(const VerticesPtr& first_poses_array,const VerticesPtr& second_poses_array);
    void renewFirstPartPoses(const VerticesPtr& first_poses_array);
    void renewSecondPartPoses(const VerticesPtr& second_poses_array);
    void renewMap(const int& initial_map_size);              
    void addScanData(const LaserScanPtr& scan,const tf::Pose& robot_pose);
    void updateMapOrigin();
    void poseEigenToTF(const Eigen::Isometry2d& pose_eigen,tf::Pose& pose_tf);

    tf::Pose origin_center,transformed_center;
    int map_expand_speed_by_pixels;
    int log_max,log_confirm_occ,log_unknown,log_confirm_free,log_min,log_free,log_occ;
    double resolution;

    bool init;
    int serial_num;
    std::shared_ptr<MapInfo> map_info;
    double origin_x,origin_y;
    int height,width,map_size;
    double min_x,min_y,max_x,max_y;
    double update_min_x,update_max_x,update_min_y,update_max_y;
    tf::Pose map_origin;

    int own_scan_num;
    std::vector<tf::Pose> scan_poses;
    std::vector<LaserScanPtr> scan_data;

    unsigned char* log_map;

    boost::mutex update_mutex[3];
    bool update;

    ros::Publisher map_pub;

    // half map
    int height_half,width_half,map_size_half;
    double origin_x_half,origin_y_half;
    unsigned char* log_map_half;

    // transformed map
    int height_transformed,width_transformed,map_size_transformed;
    double origin_x_transformed,origin_y_transformed;
    unsigned char* log_map_transformed;
    bool create_transformed_map;
};


#endif 