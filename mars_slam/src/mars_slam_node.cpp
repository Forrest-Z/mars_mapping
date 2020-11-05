#include "mars_slam/mars_slam_node.h"

MarsSlam::MarsSlam():
receive_new_scan(false),
stop_procedure(false),
stop_scan_process(false),
stop_cell_process(false),
stop_loop_closure_detection(false),
stop_optimization(false),
success_to_stop(false)
{
    nh = std::make_shared<ros::NodeHandle>("~");

    // debug 
    nh->param<bool>("debug_", debug_, false);
    if(debug_){
        ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug);
        std::cout << "\n\n\n" << RED<<"OPEN NODE IN DEBUG MODE !!" << "\n\n\n" <<RESET;
    }
    
    // init pose estimator for initial guess
    pose_estimator = std::make_unique<PoseExtrapolator>(*nh);
    // init laserscan processor
    laserscan_processor = std::make_unique<LaserscanProcessor>(*nh);
    // init motion filter
    motion_filter = std::make_unique<MotionFilter>(*nh);
    // init pose optimizer
    pose_optimizer = std::make_shared<PoseOptimizer>(*nh);
    // init map manager
    map_manager = std::make_shared<MapManager>(nh,pose_optimizer);
    // init scan matcher
    scan_matcher = std::make_shared<ScanMatcher>(nh,map_manager);
    // init loop closure detector
    loop_closure_detector = std::make_shared<LoopClosureDetection>(nh,map_manager->mapStacks(),pose_optimizer);

    paramSetting();

    // subscribers 
    scan_sub = nh->subscribe(scan_topic,1000,&MarsSlam::laserScanCallback,this);

    // services
    stop_procedure_service = nh->advertiseService("stop_procedure",&MarsSlam::stopProcedure,this);
    save_map_service = nh->advertiseService("save_map",&MarsSlam::saveMap,this);

    if(pose_estimator->openOdomSubThread())
    {
        odom_sub_thread = std::async(std::launch::async, &MarsSlam::odomLoop,this);
        ROS_INFO_STREAM("Open odom queue thread, start receiving odom msg !!");
        ROS_INFO_STREAM(" ");
    }

    // threads
    loop_closure_detection_thread = std::async(std::launch::async, &MarsSlam::loopClosureDetectionLoop,this);
    ROS_INFO_STREAM("Open loop closure thread !!");
    ROS_INFO_STREAM(" "); 

    scan_processing_thread = std::async(std::launch::async, &MarsSlam::scanProcessingLoop,this);
    ROS_INFO_STREAM("Open scan processing thread !!");
    ROS_INFO_STREAM(" "); 

    occ_map_update_thread = std::async(std::launch::async, &MarsSlam::updateMapLoop,this);
    ROS_INFO_STREAM("Open map updating thread !!");
    ROS_INFO_STREAM(" "); 

    optimization_thread = std::async(std::launch::async, &MarsSlam::optimizationLoop,this);
    ROS_INFO_STREAM("Open optimization thread !!");
    ROS_INFO_STREAM(" "); 

    while (ros::ok())
    {
        ros::Duration(0.1).sleep();
        ros::spinOnce();
    }
    
    stopProcess();
}

void MarsSlam::paramSetting()
{
    // multiple threads speed up
    nh->param<int>("omp_max_thread_num", omp_max_thread_num, 4);
    // laserscan
    nh->param<std::string>("scan_topic", scan_topic, "scan");
    // loop closure
    nh->param<double>("loop_closure/detection_frequency", loop_closure_detection_frequency, 1.0);
    nh->param<double>("loop_closure/min_dist", loop_closure_min_dist, 1.0);
    // optimization  
    nh->param<double>("optimization_frequency", optimization_frequency, 0.2);
}

void MarsSlam::laserScanCallback(const sensor_msgs::LaserScanConstPtr& scan_msg)
{
    sensor_msgs::LaserScanPtr new_scan(new sensor_msgs::LaserScan);
    *new_scan = *scan_msg;
    scan_mutex.lock();
    scan_queue.push(new_scan);
    scan_mutex.unlock();
}

void MarsSlam::scanProcessingLoop()
{
    while(ros::ok())
    {
        if(scan_queue.size() > 1)
            ROS_DEBUG_STREAM(RED <<"Scan queue size: " << scan_queue.size());
        processRangeData();
        // ros::spinOnce();

        if(stop_procedure && scan_queue.size() <= 0)
            break;
    }
    stop_scan_process = true;
    ROS_INFO_STREAM("Stop process scan data !!");
    // ros::spin();
}

void MarsSlam::processRangeData()
{
    if(scan_queue.size() <= 0)
        return;

    scan_mutex.lock();
    sensor_msgs::LaserScanPtr scan_msg = scan_queue.front();
    scan_queue.pop();
    scan_mutex.unlock();
    ros::Time now_time = scan_msg->header.stamp;

    // set scan frame id for tf 
    if(!pose_estimator->getScanFrameId())
        pose_estimator->setScanFrame(scan_msg->header.frame_id);

    // try to get initial guess
    tf::Pose guess_pose,final_pose;
    if(!pose_estimator->getInitGuess(now_time,guess_pose))
        return;
    
    // get transform from laser to robot frame
    if(!laserscan_processor->setScanCorrectedTF())
        laserscan_processor->setScanCorrectedTransform(pose_estimator->getTfToCorrectedPose());

    // transform laserscan to right frame and scan matching data type
    PCLScanCloudPtr scan_cloud(new PCLScanCloud);
    PCLScanCloudPtr scan_cloud_down(new PCLScanCloud);
    laserscan_processor->processLaserscan(scan_msg,scan_cloud,scan_cloud_down);

    // scan matching
    final_pose = scan_matcher->doScanMatching(guess_pose,scan_cloud,scan_cloud_down);

    // save cells data to queue to wait to be updated to map
    auto new_cell_data = std::make_shared<CellsData>();
    new_cell_data->cloud = scan_cloud;
    new_cell_data->cloud_down = scan_cloud_down;
    new_cell_data->pose = final_pose;
    new_cell_data->time = now_time;

    cells_data_mutex.lock();
    cells_data_queue.push(new_cell_data);
    cells_data_mutex.unlock();
    
    // update pose estimator and publish transform
    pose_estimator->updatePoseArrayAndPubTF(final_pose,now_time);
    laserscan_processor->publishPts(scan_cloud,scan_cloud_down,final_pose,now_time);
}

// odom queue for initial guess
void MarsSlam::odomLoop()
{
    pose_estimator->openOdomSub();
    ros::spin();
}

void MarsSlam::stopProcess()
{
    
    pose_optimizer->save();
    map_manager->deleteMap();
    std::cout << "\n\n\n" << BOLDYELLOW << " ~~~~~~~~~~~~ Release Memory ~~~~~~~~~~~~" << RESET << std::endl;
    std::cout << "\n\n" << BOLDYELLOW << " ~~~~~~~~~~~~~~~~ CLOSE  ~~~~~~~~~~~~~~~~" << RESET << "\n\n\n" << std::endl;
}

void MarsSlam::loopClosureDetectionLoop()
{
    ros::Rate r(loop_closure_detection_frequency);

    tf::Pose last_pose_find_loop_closure;
    bool first_loop_closure = false;

    while(ros::ok() && !stop_procedure)
    {
        if(receive_new_scan)
        {
            last_scan_mutex.lock();
            if(first_loop_closure)
            {
                if(motion_filter->dist(last_pose_find_loop_closure,last_scan.pose) > loop_closure_min_dist)
                    if(loop_closure_detector->detectLoop(last_scan))
                    {
                        last_pose_find_loop_closure = last_scan.pose;
                    }
            }else{
                if(loop_closure_detector->detectLoop(last_scan))
                {
                    last_pose_find_loop_closure = last_scan.pose;
                    first_loop_closure = true;
                }
            }
            receive_new_scan = false;
            last_scan_mutex.unlock();
        }

        ROS_DEBUG_STREAM(BOLDWHITE << "Try to detect loop closure !!");
        r.sleep();
        // ros::spinOnce();
    }

    stop_loop_closure_detection = true;
    ROS_INFO_STREAM("Stop loop closure process");
}

void MarsSlam::updateMapLoop()
{
    while(ros::ok())
    {
        if(cells_data_queue.size() > 1)
            ROS_DEBUG_STREAM(RED << "Cells data queue size: " << cells_data_queue.size());

        processCellsData();
        // ros::spinOnce();

        if(stop_procedure && cells_data_queue.size() <= 0 && stop_scan_process)
            break;
    }
    stop_cell_process = true;
    ROS_INFO_STREAM("Stop update map by cell data !!");
    // ros::spin();
}

void MarsSlam::processCellsData()
{
    if(cells_data_queue.size() <= 0)
        return;

    cells_data_mutex.lock();
    CellsDataPtr cells_data = cells_data_queue.front();
    cells_data_queue.pop();
    cells_data_mutex.unlock();

    if(motion_filter->checkUpdate(cells_data->pose,cells_data->time))
    {
        int current_submap_serial_num;
        int scan_serial_num;
       
        map_manager->dealWithNewScanData(cells_data->cloud,cells_data->pose,
                                         current_submap_serial_num,scan_serial_num);
        last_scan_mutex.lock();
        last_scan.current_submap_serial_num = current_submap_serial_num;
        last_scan.scan_serial_num = scan_serial_num;
        last_scan.cloud = cells_data->cloud_down;
        last_scan.pose = cells_data->pose;
        receive_new_scan = true;
        last_scan_mutex.unlock();
    }
}

void MarsSlam::optimizationLoop()
{
    ros::Rate r(optimization_frequency * 100);
    omp_set_num_threads(omp_max_thread_num);

    int count = 0;

    while(ros::ok())
    {
        count++;
        if(count == 100){
            count = 0;
            int last_submap_to_be_updated = pose_optimizer->doOptimization();
            if(last_submap_to_be_updated >= 0)
            {
                // update map 
                map_manager->updateSubmapByOptimizationPoses(last_submap_to_be_updated);
            }
        }
        r.sleep();
        // ros::spinOnce();

        if(stop_procedure && 
           stop_scan_process && 
           stop_cell_process && 
           stop_loop_closure_detection)
        {
            if(pose_optimizer->needToBeOptimized())
                count = 99;
            else
                break;
        }
    }    

    stop_optimization = true;
    ROS_INFO_STREAM("Finsih optimizing all map, stop optimization process !!");
}

bool MarsSlam::stopProcedure(std_srvs::Empty::Request &req ,std_srvs::Empty::Response &res)
{
    ROS_INFO_STREAM(BOLDGREEN <<"Call stop procedure !!");
    stop_procedure = true;
    scan_sub.shutdown();
    scan_matcher->callStopUpdateCloud();

    while((!stop_scan_process || !stop_cell_process || !stop_loop_closure_detection || 
           !stop_optimization || !scan_matcher->successToStop()) 
            && ros::ok())
    {
        ROS_INFO_STREAM(BOLDGREEN << "Deal with rest scan and cell data ....");
        ros::Duration(1.0).sleep();
        ros::spinOnce();
    }

    ROS_INFO_STREAM(BOLDGREEN <<"Success to stop procedure !!");
    success_to_stop = true;
    return true;
}

bool MarsSlam::saveMap(mars_srvs::SaveMap::Request &req ,mars_srvs::SaveMap::Response &res)
{
    ROS_INFO_STREAM(GREEN << "Call saving map");
    if(!success_to_stop){
        ROS_ERROR_STREAM("Process is under operating, fail to save map !!");
        return false;
    }

    map_manager->stopPubGlobalMap();
    if(map_manager->callSaveMap(req.file_path))
    {
        ROS_INFO_STREAM(GREEN << "Success to save map");
        return true;
    }else{
        return false;
    }
}
