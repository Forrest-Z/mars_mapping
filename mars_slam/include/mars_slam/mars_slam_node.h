#ifndef MARS_SLAM_NODE_H
#define MARS_SLAM_NODE_H

#include <omp.h>
#include <queue>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <std_srvs/Empty.h>
#include "mars_slam/debug.h"
#include "mars_slam/pose_extrapolator.h"
#include "mars_slam/laserscan_processor.h"
#include "mars_slam/map/map_manager.h"
#include "mars_slam/motion_filter.h"
#include "mars_slam/scan_matching/scan_matcher.h"
#include "mars_slam/loop_closure_detection/loop_closure_detection.h"
#include "mars_slam/optimization/pose_optimizer.h"

class MarsSlam
{
public:
    explicit MarsSlam();
    MarsSlam(const MarsSlam&) = delete;
    ~MarsSlam(){}
    void laserScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg);
    bool stopProcedure(std_srvs::Empty::Request &req ,std_srvs::Empty::Response &res);
    bool saveMap(mars_srvs::SaveMap::Request &req ,mars_srvs::SaveMap::Response &res);

private:
    void paramSetting();
    void scanProcessingLoop();
    void processRangeData();
    void odomLoop();
    void stopProcess();
    void loopClosureDetectionLoop();
    void updateMapLoop();
    void processCellsData();
    void optimizationLoop();

    std::shared_ptr<ros::NodeHandle> nh;
    // estimator of pose
    std::unique_ptr<PoseExtrapolator> pose_estimator;
    // laserscan processor
    std::unique_ptr<LaserscanProcessor> laserscan_processor;
    // motion filter
    std::unique_ptr<MotionFilter> motion_filter;
    // map manager
    std::shared_ptr<MapManager> map_manager;
    // scan matcher
    std::shared_ptr<ScanMatcher> scan_matcher;
    // loop closure
    std::shared_ptr<LoopClosureDetection> loop_closure_detector;
    // pose optimization
    std::shared_ptr<PoseOptimizer> pose_optimizer;
    
    // publisher & subscriber
    ros::Subscriber scan_sub;

    // param
    bool debug_;
    int omp_max_thread_num;
    std::string scan_topic;
    double loop_closure_detection_frequency;

    // scan queue
    std::queue<sensor_msgs::LaserScanPtr> scan_queue;
    boost::mutex scan_mutex;

    // cells data queue
    std::queue<CellsDataPtr> cells_data_queue;
    boost::mutex cells_data_mutex;

    // loop closure scan
    InsertedCellsData last_scan;
    bool receive_new_scan;
    double loop_closure_min_dist;
    boost::mutex last_scan_mutex;

    // optimization
    double optimization_frequency;

    // threads
    std::future<void> odom_sub_thread;
    std::future<void> scan_processing_thread;
    std::future<void> loop_closure_detection_thread;
    std::future<void> occ_map_update_thread;
    std::future<void> optimization_thread;

    // stop
    ros::ServiceServer stop_procedure_service;
    bool stop_procedure;
    bool stop_scan_process;
    bool stop_cell_process;
    bool stop_loop_closure_detection;
    bool stop_optimization;
    bool success_to_stop;

    // save map
    ros::ServiceServer save_map_service;

};

#endif 