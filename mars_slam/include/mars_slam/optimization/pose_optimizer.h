#ifndef POSE_OPTIMIZER_H
#define POSE_OPTIMIZER_H

#include <boost/thread/mutex.hpp>
#include <ros/ros.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/transform_datatypes.h>
#include <visualization_msgs/MarkerArray.h>
#include <g2o/core/factory.h>
#include <g2o/types/slam3d/vertex_se3.h>
#include <g2o/types/slam3d/edge_se3.h>
#include <g2o/types/slam2d/vertex_se2.h>
#include <g2o/types/slam2d/edge_se2.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/sparse_optimizer.h>
#include "g2o/core/optimization_algorithm_gauss_newton.h"
// #include "g2o/core/optimization_algorithm_levenberg.h"
// #include "g2o/types/sba/types_six_dof_expmap.h"
#include "mars_slam/debug.h"
#include "mars_slam/optimization/simple_graph.h"

// typedef std::vector<g2o::VertexSE3*> Vertices;
// typedef std::shared_ptr<std::vector<g2o::VertexSE3*>> VerticesPtr;
typedef std::vector<g2o::VertexSE2*> Vertices;
typedef std::shared_ptr<std::vector<g2o::VertexSE2*>> VerticesPtr;

typedef struct
{
    tf::Pose pose_on_target_submap;
    tf::Pose target_submap_center;
    int target_submap_serial_num;
    int source_submap_serial_num;
    int source_scan_serial_num;
    Eigen::Isometry2d T;

}LoopClosureData;

class PoseOptimizer
{
public:
    explicit PoseOptimizer(){}
    explicit PoseOptimizer(ros::NodeHandle& nh_);
    ~PoseOptimizer(){}
    void addNewVertex(const int& submap_num,const tf::Pose& new_pose);
    void save();
    // void addNewGlobalEdge(const tf::Pose& pose_on_submap,
    //                       const tf::Pose& center_on_submap,
    //                       const int& source_submap_serial_num,
    //                       const int& source_scan_serial_num,
    //                       const int& target_submap_serial_num);
    void addNewGlobalEdge(std::queue<LoopClosureData>& new_loop_closure_data_queue);
    void pubConstraintMarker();
    int doOptimization();
    const VerticesPtr& getVertices(const int& n){return vertices_array[n];}
    bool needToBeOptimized(){return need_to_be_optimized;}
    
private:
    void initOptimizer();
    void setInformationMatrix();
    // void addNewVertexToArray(g2o::VertexSE3* new_vertex,const int& submap_num);
    void addNewVertexToArray(g2o::VertexSE2* new_vertex,const int& submap_num);
    void cancelFix(const int& submap_num);
    // void addNewLocalEdge(g2o::VertexSE3* new_vertex,const tf::Pose& new_pose_tf);
    void addNewLocalEdge(g2o::VertexSE2* new_vertex,const tf::Pose& new_pose_tf);
    void initConstraintMarker();
    void addNewConstraintMarker(const tf::Pose& p1 , const tf::Pose& p2,const int& type);
    void updateConstraintMarkers();
    void updateLocalConstraintMarkers();
    void updateGlobalConstraintMarkers();
    double dist(const tf::Pose& a,const tf::Pose& b);
    double dist(const Eigen::Isometry2d& a,const Eigen::Isometry2d& b);
    void poseTFToEigen(const tf::Pose& pose_tf,Eigen::Isometry2d& pose_eigen);
    void poseEigenToTF(const Eigen::Isometry2d& pose_eigen,tf::Pose& pose_tf);
    void pointEigenToMsg(const Eigen::Isometry2d& pose_eigen,geometry_msgs::Point& msg);
    std_msgs::ColorRGBA blue();
    std_msgs::ColorRGBA yellow();

    ros::NodeHandle nh;

    std::vector<VerticesPtr> vertices_array;
    Vertices vertices;
    // std::vector<g2o::EdgeSE3*> local_edges;
    // std::vector<g2o::EdgeSE3*> global_edges;
    std::vector<g2o::EdgeSE2*> local_edges;
    std::vector<g2o::EdgeSE2*> global_edges;
    // g2o::VertexSE3* pre_vertex;
    g2o::VertexSE2* pre_vertex;
    int id_count;
    bool need_to_be_optimized;

    // pose
    tf::Pose last_pose_tf;
    // Eigen::Isometry3d last_pose_eigen;
    int long_neighbor_threshold;
    double pose_max_update_dist_short;
    double pose_max_update_dist_long;
    double compare_min_dis;
    double min_error_between_two_loop_closure;

    // g2o optimizer
    g2o::SparseOptimizer optimizer;

    // information_matrix
    // Eigen::Matrix<double, 6, 6> local_information;
    // Eigen::Matrix<double, 6, 6> global_information;
    Eigen::Matrix<double, 3, 3> local_information;
    Eigen::Matrix<double, 3, 3> global_information;

    // visualization
    ros::Publisher local_constraint_markers_pub;
    ros::Publisher global_constraint_markers_pub;
    visualization_msgs::Marker local_constraint_marker;
    visualization_msgs::Marker global_constraint_marker;
    boost::mutex local_constraint_marker_mutex;
    boost::mutex global_constraint_marker_mutex;

    boost::mutex update_vertices_mutex;

    // graph for submaps
    std::unique_ptr<SimpleGraph> submaps_graph;
    std::queue<NeighborPair> neighbors_array;

    std::vector<LoopClosureData> loop_closure_data_queue;

};

#endif 