#ifndef LOOP_CLOSURE_DETECTION_H 
#define LOOP_CLOSURE_DETECTION_H 

// #include <future>
#include <queue>
#include <ros/ros.h>
#include <boost/thread/mutex.hpp>
#include <tf/transform_datatypes.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "mars_slam/debug.h"
#include "mars_slam/scan.h"
#include "mars_slam/loop_closure_detection/map_stacks.h"
#include "mars_slam/optimization/pose_optimizer.h"
#include "mars_slam/map/map.h"

struct PoseNode 
{
    int x;
    int y;
    int level;
    double score;

    PoseNode(int level_,double score_,int x_,int y_) {level = level_;score = score_;x = x_;y = y_;}
    bool operator<(const PoseNode& a) const
    {
        if(level != a.level)
            return level > a.level;
        else      
            return score < a.score; 
    }
};

class LoopClosureDetection
{
public:
    explicit LoopClosureDetection(){}
    explicit LoopClosureDetection(std::shared_ptr<ros::NodeHandle>& nh_,
                                  std::shared_ptr<MapStacks>& map_stacks_,
                                  std::shared_ptr<PoseOptimizer>& pose_optimizer_);
    LoopClosureDetection(const LoopClosureDetection&) = delete;
    ~LoopClosureDetection(){}
    bool detectLoop(const InsertedCellsData& cells_data);
    

private:
    void decideSubmapsToBeSearched(const tf::Pose& pose,std::vector<int>& submaps_to_be_searched);
    double dist(const tf::Pose& a,const tf::Pose& b);
    bool branchAndBound(const InsertedCellsData& cells_data,const int& stack_serial_num,
                              LoopClosureData& loop_closure_data);
    Eigen::Matrix3d getRotationMatrix(const double& theta);
    void transPoints(const Eigen::Matrix3d& poseMat,const PCLScanCloudPtr& scan_cloud,
                           std::vector<Eigen::Vector2i>& pts_tranformed,const double& resolution);    
    Eigen::Vector2d transPoint(const pcl::PointXYZ& pt,const Eigen::Matrix3d& T);
    void computeHighestLevelScore(std::priority_queue<PoseNode>& score_queue,
                                 const OccGridStackPtr& map_stack,
                                 const std::vector<Eigen::Vector2i>& pts_tranformed);
    double calculateScore(const std::vector<Eigen::Vector2i>& points,
                          const nav_msgs::OccupancyGridPtr& map,const int& x,const int& y);
    bool compute(std::priority_queue<PoseNode>& score_queue,const OccGridStackPtr& map_stack,
                const std::vector<Eigen::Vector2i>& points, double& max_score, Eigen::Vector2i& max_score_pose);
                                       

    std::shared_ptr<ros::NodeHandle> nh; 
    std::shared_ptr<MapStacks> map_stacks;

    // param
    double search_submap_max_dist,theta_resolution,min_score;
    double theta_search_window_size,theta_search_window_radius;
    int branch_and_bound_depth,highest_level;
    int theta_slices_num;
    double theta_resolution_degree,theta_resolution_rad;

    // pose optimizer
    std::shared_ptr<PoseOptimizer> pose_optimizer;

};

#endif 