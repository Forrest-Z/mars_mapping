#include "mars_slam/optimization/pose_optimizer.h"

PoseOptimizer::PoseOptimizer(ros::NodeHandle& nh_):nh(nh_),id_count(0),need_to_be_optimized(false)
{
    submaps_graph = std::make_unique<SimpleGraph>();

    initOptimizer();
    setInformationMatrix();
    initConstraintMarker();
    local_constraint_markers_pub = nh.advertise<visualization_msgs::Marker>("local_constraint",1,true);
    global_constraint_markers_pub = nh.advertise<visualization_msgs::Marker>("global_constraint",1,true);

    nh.param<int>("loop_closure/long_neighbor_threshold",long_neighbor_threshold, 3);
    nh.param<double>("loop_closure/pose_max_update_dist_short",pose_max_update_dist_short, 0.5);
    nh.param<double>("loop_closure/pose_max_update_dist_long",pose_max_update_dist_long, 5.0);
    nh.param<double>("loop_closure/compare_min_dis",compare_min_dis, 0.2);
    nh.param<double>("loop_closure/min_error_between_two_loop_closure",min_error_between_two_loop_closure, 0.05);
}

void PoseOptimizer::initOptimizer()
{
    auto linear_solver = g2o::make_unique<g2o::LinearSolverCholmod<g2o::BlockSolverX::PoseMatrixType>>();
    auto block_solver = g2o::make_unique<g2o::BlockSolverX>(std::move(linear_solver));
    auto optimization_algorithm = new g2o::OptimizationAlgorithmGaussNewton(std::move(block_solver));
    // auto optimization_algorithm = new g2o::OptimizationAlgorithmLevenberg(std::move(block_solver));

    optimizer.setVerbose(false);
    optimizer.setAlgorithm(optimization_algorithm);
}

void PoseOptimizer::setInformationMatrix()
{
    // local_information = Eigen::Matrix<double, 6, 6>::Zero();
    // local_information.block<3,3>(0,0) = 20 * Eigen::Matrix3d::Identity();
    // local_information.block<3,3>(3,3) = 100 * Eigen::Matrix3d::Identity();
    // global_information = Eigen::Matrix<double, 6, 6>::Zero();
    // global_information.block<3,3>(0,0) = 100 * Eigen::Matrix3d::Identity();
    // global_information.block<3,3>(3,3) = 1000 * Eigen::Matrix3d::Identity();
    // local_information = Eigen::Matrix<double, 6, 6>::Zero();
    // local_information.block<3,3>(0,0) = 10 * Eigen::Matrix3d::Identity();
    // local_information.block<3,3>(3,3) = 1000 * Eigen::Matrix3d::Identity();
    // global_information = Eigen::Matrix<double, 6, 6>::Zero();
    // global_information.block<3,3>(0,0) = 1000000 * Eigen::Matrix3d::Identity();
    // global_information.block<3,3>(3,3) = 1000000 * Eigen::Matrix3d::Identity();
    // local_information = Eigen::Matrix<double, 3, 3>::Zero();
    // local_information.block<2,2>(0,0) = 10 * Eigen::Matrix2d::Identity();
    // local_information(2,2) = 1000; 
    // global_information = Eigen::Matrix<double, 3, 3>::Zero();
    // global_information.block<2,2>(0,0) = 1000000 * Eigen::Matrix2d::Identity();
    // global_information(2,2) = 1000000;
    local_information = Eigen::Matrix<double, 3, 3>::Zero();
    local_information.block<2,2>(0,0) = 100 * Eigen::Matrix2d::Identity();
    local_information(2,2) = 1000; 
    global_information = Eigen::Matrix<double, 3, 3>::Zero();
    global_information.block<2,2>(0,0) = 1000000 * Eigen::Matrix2d::Identity();
    global_information(2,2) = 1000000;
}

void PoseOptimizer::addNewVertex(const int& submap_num,const tf::Pose& pose_tf)
{
    // Eigen::Isometry3d pose_eigen;
    // tf::poseTFToEigen(pose_tf,pose_eigen);

    Eigen::Isometry2d pose_eigen;
    poseTFToEigen(pose_tf,pose_eigen);

    // g2o::VertexSE3* new_vertex = new g2o::VertexSE3;
    g2o::VertexSE2* new_vertex = new g2o::VertexSE2;
    new_vertex->setId(id_count);
    new_vertex->setEstimate(pose_eigen);
    new_vertex->setFixed(true);

    update_vertices_mutex.lock();
    addNewVertexToArray(new_vertex,submap_num);
    addNewLocalEdge(new_vertex,pose_tf);
    update_vertices_mutex.unlock();

    id_count++;
}

void PoseOptimizer::addNewVertexToArray(g2o::VertexSE2* new_vertex,const int& submap_num)
// void PoseOptimizer::addNewVertexToArray(g2o::VertexSE3* new_vertex,const int& submap_num)
{
    if(submap_num >= vertices_array.size())
    {
        auto new_vertices = std::make_shared<Vertices>();
        vertices_array.push_back(new_vertices);

        if(vertices_array.size() > 2)
            cancelFix(vertices_array.size()-3);

        if(submap_num > 0)
            submaps_graph->addEdge(submap_num - 1,submap_num);
    }

    vertices_array[submap_num]->push_back(new_vertex);
    vertices.push_back(new_vertex);
    optimizer.addVertex(new_vertex);
}

void PoseOptimizer::cancelFix(const int& submap_num)
{
    int vertices_num = vertices_array[submap_num]->size();

    for(unsigned int i = 0;i < vertices_num;i++)
        (*vertices_array[submap_num])[i]->setFixed(false);
}

void PoseOptimizer::addNewLocalEdge(g2o::VertexSE2* new_vertex,const tf::Pose& new_pose_tf)
// void PoseOptimizer::addNewLocalEdge(g2o::VertexSE3* new_vertex,const tf::Pose& new_pose_tf)
{
    if(id_count > 0)
    {
        // g2o::EdgeSE3* new_edge = new g2o::EdgeSE3;
        g2o::EdgeSE2* new_edge = new g2o::EdgeSE2;

        tf::Transform T = last_pose_tf.inverse()*new_pose_tf;
        // Eigen::Isometry3d T_;
        // tf::transformTFToEigen(T,T_);
        Eigen::Isometry2d T_;
        poseTFToEigen(T,T_);

        new_edge->setVertex(0,pre_vertex);
        new_edge->setVertex(1,new_vertex);
        new_edge->setMeasurement(T_);
        new_edge->setInformation(local_information);
        local_edges.push_back(new_edge);
        optimizer.addEdge(new_edge);

        addNewConstraintMarker(last_pose_tf,new_pose_tf,1);
    }

    pre_vertex = new_vertex;
    last_pose_tf = new_pose_tf;
}

// void PoseOptimizer::addNewGlobalEdge(const tf::Pose& pose_on_submap,
//                                      const tf::Pose& center_on_submap,
//                                      const int& source_submap_serial_num,
//                                      const int& source_scan_serial_num,
//                                      const int& target_submap_serial_num)
void PoseOptimizer::addNewGlobalEdge(std::queue<LoopClosureData>& new_loop_closure_data_queue)
{
    update_vertices_mutex.lock();   

    while(new_loop_closure_data_queue.size() > 0)
    {
        LoopClosureData new_loop_closure_data = new_loop_closure_data_queue.front();
        new_loop_closure_data_queue.pop();

        int target_submap_serial_num = new_loop_closure_data.target_submap_serial_num;
        int source_submap_serial_num = new_loop_closure_data.source_submap_serial_num;
        int source_scan_serial_num = new_loop_closure_data.source_scan_serial_num;
        tf::Pose pose_on_submap = new_loop_closure_data.pose_on_target_submap;
        tf::Pose center_on_submap = new_loop_closure_data.target_submap_center;

        g2o::VertexSE2* pre = (*vertices_array[target_submap_serial_num])[0];
        g2o::VertexSE2* cur = (*vertices_array[source_submap_serial_num])[source_scan_serial_num];
        Eigen::Isometry2d pose_on_submap_eigen;
        Eigen::Isometry2d center_on_submap_eigen;
        poseTFToEigen(pose_on_submap,pose_on_submap_eigen);
        poseTFToEigen(center_on_submap,center_on_submap_eigen);
        Eigen::Isometry2d T = center_on_submap_eigen.inverse()*pose_on_submap_eigen;

        // to detect error loop closure
        int submap_dist = submaps_graph->dist(source_submap_serial_num,target_submap_serial_num);
        double pose_max_update_dist;

        if(submap_dist >= long_neighbor_threshold)
            pose_max_update_dist = pose_max_update_dist_long;
        else
            pose_max_update_dist = pose_max_update_dist_short;

        tf::Pose cur_pose_true;
        poseEigenToTF(cur->estimate().toIsometry(),cur_pose_true);
        tf::Pose cur_pose_estimate;
        poseEigenToTF(pre->estimate().toIsometry()*T,cur_pose_estimate);
        double pose_dist = dist(cur_pose_estimate,cur_pose_true);

        if(pose_dist >= pose_max_update_dist)
        {
            ROS_INFO_STREAM(YELLOW<<"Detect error loop closure !! "<< pose_dist << " >= " << pose_max_update_dist);
        }else{
            ROS_INFO_STREAM(GREEN<<"Loop closure !! "<< pose_dist << " < " << pose_max_update_dist);
            new_loop_closure_data.T = T;
            loop_closure_data_queue.push_back(new_loop_closure_data);
        }

    }

    
    // while(loop_closure_data_queue.size() > 0)
    // {
    //     LoopClosureData loop_closure_data = loop_closure_data_queue.front();
    //     loop_closure_data_queue.pop();
    //     int target_submap_serial_num = loop_closure_data.target_submap_serial_num;
    //     int source_submap_serial_num = loop_closure_data.source_submap_serial_num; 
    //     int source_scan_serial_num = loop_closure_data.source_scan_serial_num;  

    //     // submaps_graph->addEdge(source_submap_serial_num,target_submap_serial_num);
    //     NeighborPair neighbor_pair;
    //     neighbor_pair.a = source_submap_serial_num;
    //     neighbor_pair.b = target_submap_serial_num;
    //     neighbors_array.push(neighbor_pair);

    //     g2o::VertexSE2* pre = (*vertices_array[target_submap_serial_num])[0];
    //     g2o::VertexSE2* cur = (*vertices_array[source_submap_serial_num])[source_scan_serial_num];
    //     Eigen::Isometry2d T = loop_closure_data.T;

    //     g2o::EdgeSE2* new_edge = new g2o::EdgeSE2;
    //     new_edge->setVertex(0,pre);
    //     new_edge->setVertex(1,cur);   
    //     new_edge->setMeasurement(T);
    //     new_edge->setInformation(global_information);
    //     global_edges.push_back(new_edge);
    //     optimizer.addEdge(new_edge);
    //     tf::Pose pre_pose_tf,cur_pose_tf;

    //     poseEigenToTF(pre->estimate().toIsometry(),pre_pose_tf);
    //     poseEigenToTF(cur->estimate().toIsometry(),cur_pose_tf);
    //     addNewConstraintMarker(pre_pose_tf,cur_pose_tf,2);
    //     need_to_be_optimized = true;
    // }
    
    int first_count = 0;
    int count = 1;
    // int count = 0;
    // while(loop_closure_data_queue.size() >= (2+count))
    while(loop_closure_data_queue.size() >= 2 && 
         (first_count+count < loop_closure_data_queue.size()) && 
          ros::ok())
    {
        // LoopClosureData first = loop_closure_data_queue.front();
        LoopClosureData first = loop_closure_data_queue[first_count];
        // loop_closure_data_queue.pop();

        int first_target_submap_serial_num = first.target_submap_serial_num;
        int first_source_submap_serial_num = first.source_submap_serial_num; 
        int first_source_scan_serial_num = first.source_scan_serial_num;

        g2o::VertexSE2* pre_first = (*vertices_array[first_target_submap_serial_num])[0];
        g2o::VertexSE2* cur_first = (*vertices_array[first_source_submap_serial_num])[first_source_scan_serial_num];
        Eigen::Isometry2d T_first = first.T; 

        // LoopClosureData second = loop_closure_data_queue.front();
        LoopClosureData second = loop_closure_data_queue[first_count+count];
        // loop_closure_data_queue.pop();

        int second_target_submap_serial_num = second.target_submap_serial_num;
        int second_source_submap_serial_num = second.source_submap_serial_num; 
        int second_source_scan_serial_num = second.source_scan_serial_num;

        g2o::VertexSE2* pre_second = (*vertices_array[second_target_submap_serial_num])[0];
        g2o::VertexSE2* cur_second = (*vertices_array[second_source_submap_serial_num])[second_source_scan_serial_num];
        Eigen::Isometry2d T_second = second.T; 

        double two_loop_closure_dist_true = dist(cur_first->estimate().toIsometry(),
                                                 cur_second->estimate().toIsometry());
        // ROS_INFO_STREAM(two_loop_closure_dist_true);
        if(two_loop_closure_dist_true  < compare_min_dis || 
           first_target_submap_serial_num != second_target_submap_serial_num)
        {
            count++; 
            if(first_count + count == loop_closure_data_queue.size())
            {
                first_count++;
                count = 1;
            }
            continue;
        }else{
            count = 1;
            loop_closure_data_queue.erase(loop_closure_data_queue.begin()+first_count);
        }

        double two_loop_closure_dist_esti = dist(pre_first->estimate().toIsometry()*T_first,
                                                 pre_second->estimate().toIsometry()*T_second);
        double error = fabs(two_loop_closure_dist_true - two_loop_closure_dist_esti);

        if(error > min_error_between_two_loop_closure)
        {
            ROS_INFO_STREAM(YELLOW<<"Detect error loop closure by compare two loop closure!!  Error:" 
                                  << error);
            continue;
        }else{
            ROS_INFO_STREAM(GREEN<<"Add constraint !! Error:" << error);
        }    

        NeighborPair neighbor_pair;
        neighbor_pair.a = first_source_submap_serial_num;
        neighbor_pair.b = first_target_submap_serial_num;
        neighbors_array.push(neighbor_pair);

        g2o::EdgeSE2* new_edge = new g2o::EdgeSE2;
        new_edge->setVertex(0,pre_first);
        new_edge->setVertex(1,cur_first);   
        new_edge->setMeasurement(T_first);
        new_edge->setInformation(global_information);
        global_edges.push_back(new_edge);
        optimizer.addEdge(new_edge);
        tf::Pose pre_pose_tf,cur_pose_tf;

        poseEigenToTF(pre_first->estimate().toIsometry(),pre_pose_tf);
        poseEigenToTF(cur_first->estimate().toIsometry(),cur_pose_tf);
        addNewConstraintMarker(pre_pose_tf,cur_pose_tf,2);
        need_to_be_optimized = true;

    }

    update_vertices_mutex.unlock();   
}


void PoseOptimizer::save()
{
    std::cout << "Save pose graph !!" << std::endl;
    optimizer.save("/home/ponpon/catkin_ws/mars_slam.g2o");
}

void PoseOptimizer::initConstraintMarker()
{
    local_constraint_marker.header.frame_id = "map";
    local_constraint_marker.header.stamp = ros::Time();
    local_constraint_marker.ns = "";
    local_constraint_marker.type = visualization_msgs::Marker::LINE_LIST;
    local_constraint_marker.action = visualization_msgs::Marker::ADD;
    local_constraint_marker.id = -1;
    local_constraint_marker.scale.x = 0.03;
    local_constraint_marker.pose.orientation.w = 1.0;
    local_constraint_marker.color = blue();

    global_constraint_marker = local_constraint_marker;
    global_constraint_marker.color = yellow();
}

void PoseOptimizer::pubConstraintMarker()
{
    local_constraint_marker_mutex.lock();
    local_constraint_markers_pub.publish(local_constraint_marker);
    local_constraint_marker_mutex.unlock();
    global_constraint_marker_mutex.lock();
    global_constraint_markers_pub.publish(global_constraint_marker);
    global_constraint_marker_mutex.unlock();
}

void PoseOptimizer::addNewConstraintMarker(const tf::Pose& p1 , const tf::Pose& p2,const int& type)
{
    geometry_msgs::Point p1_,p2_;

    tf::pointTFToMsg(p1.getOrigin(),p1_);
    tf::pointTFToMsg(p2.getOrigin(),p2_);
    if(type == 1){
        local_constraint_marker_mutex.lock();
        local_constraint_marker.points.push_back(p1_);
        local_constraint_marker.points.push_back(p2_);
        local_constraint_marker_mutex.unlock();
    }else if(type == 2)
    {
        global_constraint_marker_mutex.lock();
        global_constraint_marker.points.push_back(p1_);
        global_constraint_marker.points.push_back(p2_);
        global_constraint_marker_mutex.unlock();
    }
}

int PoseOptimizer::doOptimization()
{
    if(!need_to_be_optimized)
        return -1;

    update_vertices_mutex.lock();   
    while(neighbors_array.size() > 0)
    {
        int a = neighbors_array.front().a;
        int b = neighbors_array.front().b;
        submaps_graph->addEdge(a,b);
        neighbors_array.pop();
    }

    optimizer.initializeOptimization();
    optimizer.optimize(5);  
    updateConstraintMarkers();
    ROS_INFO_STREAM("Finish optimization !!");
    update_vertices_mutex.unlock();
    need_to_be_optimized = false;

    // last subamp needed to be updated
    return vertices_array.size() - 3;
}

void PoseOptimizer::updateConstraintMarkers()
{
    updateLocalConstraintMarkers();
    updateGlobalConstraintMarkers();
}

void PoseOptimizer::updateLocalConstraintMarkers()
{
    local_constraint_marker_mutex.lock();
    local_constraint_marker.points.clear();

    for(unsigned int i = 0;i < local_edges.size();i++)
    {
        geometry_msgs::Point p1,p2;
        int id_1 = local_edges[i]->vertex(0)->id();
        int id_2 = local_edges[i]->vertex(1)->id();
        // Eigen::Isometry3d pose_eigen_1 = vertices[id_1]->estimate();
        // Eigen::Isometry3d pose_eigen_2 = vertices[id_2]->estimate();
        Eigen::Isometry2d pose_eigen_1 = vertices[id_1]->estimate().toIsometry();
        Eigen::Isometry2d pose_eigen_2 = vertices[id_2]->estimate().toIsometry();
        // tf::pointEigenToMsg(pose_eigen_1.translation(),p1);
        // tf::pointEigenToMsg(pose_eigen_2.translation(),p2);
        pointEigenToMsg(pose_eigen_1,p1);
        pointEigenToMsg(pose_eigen_2,p2);
        local_constraint_marker.points.push_back(p1);
        local_constraint_marker.points.push_back(p2);
    }
    local_constraint_marker_mutex.unlock();
}
        
void PoseOptimizer::updateGlobalConstraintMarkers()
{
    global_constraint_marker_mutex.lock();
    global_constraint_marker.points.clear();

    for(unsigned int i = 0;i < global_edges.size();i++)
    {
        geometry_msgs::Point p1,p2;
        int id_1 = global_edges[i]->vertex(0)->id();
        int id_2 = global_edges[i]->vertex(1)->id();
        // Eigen::Isometry3d pose_eigen_1 = vertices[id_1]->estimate();
        // Eigen::Isometry3d pose_eigen_2 = vertices[id_2]->estimate();
        Eigen::Isometry2d pose_eigen_1 = vertices[id_1]->estimate().toIsometry();
        Eigen::Isometry2d pose_eigen_2 = vertices[id_2]->estimate().toIsometry();
        // tf::pointEigenToMsg(pose_eigen_1.translation(),p1);
        // tf::pointEigenToMsg(pose_eigen_2.translation(),p2);
        pointEigenToMsg(pose_eigen_1,p1);
        pointEigenToMsg(pose_eigen_2,p2);
        global_constraint_marker.points.push_back(p1);
        global_constraint_marker.points.push_back(p2);
    }
    global_constraint_marker_mutex.unlock();
}

double PoseOptimizer::dist(const tf::Pose& a,const tf::Pose& b)
{
    double dx = a.getOrigin().x() - b.getOrigin().x();
    double dy = a.getOrigin().y() - b.getOrigin().y();
    return sqrt(pow(dx,2) + pow(dy,2));
}

double PoseOptimizer::dist(const Eigen::Isometry2d& a,const Eigen::Isometry2d& b)
{
    double dx = a.matrix()(6) - b.matrix()(6);
    double dy = a.matrix()(7) - b.matrix()(7);
    return sqrt(pow(dx,2) + pow(dy,2));
}

void PoseOptimizer::poseTFToEigen(const tf::Pose& pose_tf,Eigen::Isometry2d& pose_eigen)
{
    double x = pose_tf.getOrigin().x();
    double y = pose_tf.getOrigin().y();
    double theta = tf::getYaw(pose_tf.getRotation());

    Eigen::Vector2d trans =  Eigen::Vector2d(x, y);
    Eigen::Matrix2d rot ;
    rot << cos(theta),-sin(theta),
            sin(theta),cos(theta);

    pose_eigen = Eigen::Isometry2d::Identity();
    pose_eigen.translate(trans);
    pose_eigen.rotate(rot);
}

void PoseOptimizer::poseEigenToTF(const Eigen::Isometry2d& pose_eigen,tf::Pose& pose_tf)
{
    double x = pose_eigen.matrix()(6);
    double y = pose_eigen.matrix()(7);
    double theta =  atan2(pose_eigen.matrix()(1),pose_eigen.matrix()(0));
    tf::Quaternion q;
    q.setRPY(0.0,0.0,theta);

    pose_tf.setOrigin(tf::Vector3(x,y,0.0));
    pose_tf.setRotation(q);
}

void PoseOptimizer::pointEigenToMsg(const Eigen::Isometry2d& pose_eigen,geometry_msgs::Point& msg)
{
    msg.x = pose_eigen.matrix()(6);
    msg.y = pose_eigen.matrix()(7);
    msg.z = 0.0;
}

std_msgs::ColorRGBA PoseOptimizer::blue()
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;color.r = 0.0;color.g = 0.0;color.b = 1.0;
    return color;
}

std_msgs::ColorRGBA PoseOptimizer::yellow()
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;color.r = 1.0;color.g = 0.843;color.b = 0.0;
    return color;
}

