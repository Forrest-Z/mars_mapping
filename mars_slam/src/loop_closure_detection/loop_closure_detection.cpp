#include "mars_slam/loop_closure_detection/loop_closure_detection.h"

LoopClosureDetection::LoopClosureDetection(std::shared_ptr<ros::NodeHandle>& nh_,
                                           std::shared_ptr<MapStacks>& map_stacks_,
                                           std::shared_ptr<PoseOptimizer>& pose_optimizer_):
nh(nh_),
map_stacks(map_stacks_),
pose_optimizer(pose_optimizer_)
{
    nh->param<double>("loop_closure/search_submap_max_dist", search_submap_max_dist, 10.0);
    nh->param<int>("loop_closure/branch_and_bound_depth", branch_and_bound_depth, 7);
    nh->param<double>("loop_closure/theta_resolution", theta_resolution, 1.0);
    nh->param<double>("loop_closure/theta_search_window_size", theta_search_window_size, 30.0);
    nh->param<double>("loop_closure/min_score", min_score, 0.6);

    theta_search_window_radius = theta_search_window_size/2;
    theta_slices_num = ceil(theta_search_window_size/theta_resolution);
    theta_resolution_degree = theta_resolution;
    theta_resolution_rad = theta_resolution_degree/180*M_PI;
    highest_level = branch_and_bound_depth - 1;    
}

bool LoopClosureDetection::detectLoop(const InsertedCellsData& cells_data)
{
    std::vector<int> submaps_to_be_searched;
    decideSubmapsToBeSearched(cells_data.pose,submaps_to_be_searched);

    bool find_loop_closure = false;
    std::queue<LoopClosureData> loop_closure_data_queue;
    for(unsigned int i = 0;i < submaps_to_be_searched.size();i++){
        LoopClosureData new_loop_closure_data;
        if(branchAndBound(cells_data,submaps_to_be_searched[i],new_loop_closure_data)){
            find_loop_closure = true;
            loop_closure_data_queue.push(new_loop_closure_data);
        }
    }

    if(find_loop_closure)
    {
        pose_optimizer->addNewGlobalEdge(loop_closure_data_queue);
    }
            
    return find_loop_closure;
}

void LoopClosureDetection::decideSubmapsToBeSearched(const tf::Pose& pose,std::vector<int>& submaps_to_be_searched)
{
    for(unsigned int i = 0;i < map_stacks->numOfStacks();i++)
    {
        // tf::Pose center = map_stacks->getCenter(i);
        tf::Pose center = map_stacks->getTransformedCenter(i);
        if(dist(pose,center) <= search_submap_max_dist)
            submaps_to_be_searched.push_back(i);
    }
}

double LoopClosureDetection::dist(const tf::Pose& a,const tf::Pose& b)
{
    double dx = a.getOrigin().x() - b.getOrigin().x();
    double dy = a.getOrigin().y() - b.getOrigin().y();
    return sqrt(pow(dx,2) + pow(dy,2));
}

bool LoopClosureDetection::branchAndBound(const InsertedCellsData& cells_data,const int& stack_serial_num,
                                                LoopClosureData& loop_closure_data)
{
    tf::Pose pose = cells_data.pose;
    PCLScanCloudPtr scan = cells_data.cloud;

    OccGridStackPtr map_stack = map_stacks->getStack(stack_serial_num);
    int points_num = scan->points.size();
    double max_score = min_score;
    Eigen::Vector2i max_score_pose;
    double max_score_theta;
    double start_theta = 
            tf::getYaw((map_stacks->getTransformedCenter(stack_serial_num)*
                        map_stacks->getCenter(stack_serial_num).inverse()*
                        pose).getRotation()) - 
            (theta_search_window_radius/180*M_PI);
    bool find_loop_closure = false;

    double origin_x = (*map_stack)[0]->info.origin.position.x;
    double origin_y = (*map_stack)[0]->info.origin.position.y;
    double resolution = (*map_stack)[0]->info.resolution;
    int width = (*map_stack)[0]->info.width;
    int height = (*map_stack)[0]->info.height;

    for(unsigned int i = 0;i < theta_slices_num; i++)
    {
        double theta = start_theta + (double)i * theta_resolution_rad;

        // transfrom points by orientation
        Eigen::Matrix3d pose_mat = getRotationMatrix(theta);
        std::vector<Eigen::Vector2i> pts_tranformed;
        transPoints(pose_mat,scan,pts_tranformed,resolution);

        std::priority_queue<PoseNode> score_queue;
        computeHighestLevelScore(score_queue,map_stack,pts_tranformed);
               
        // branch and bound main process
        if(compute(score_queue,map_stack,pts_tranformed,max_score,max_score_pose)){
            max_score_theta = theta;
            find_loop_closure = true;
        }
    }  

    if(find_loop_closure)
    {
        ROS_INFO_STREAM(GREEN<<"Find loop closure to submap " << stack_serial_num << " , score: " << max_score);

        double final_x = origin_x + (double)max_score_pose(0) * resolution;
        double final_y = origin_y + (double)max_score_pose(1) * resolution;

        tf::Pose final_pose;
        tf::Quaternion q;

        final_pose.setOrigin(tf::Vector3(final_x,final_y,0));
        q.setRPY(0.0,0.0,max_score_theta);
        final_pose.setRotation(q);

        // pose_optimizer->addNewGlobalEdge(final_pose,map_stacks->getCenter(stack_serial_num),
        //         cells_data.current_submap_serial_num,cells_data.scan_serial_num,stack_serial_num);
        loop_closure_data.pose_on_target_submap = final_pose;
        loop_closure_data.source_scan_serial_num = cells_data.scan_serial_num;
        loop_closure_data.source_submap_serial_num = cells_data.current_submap_serial_num;
        loop_closure_data.target_submap_center = map_stacks->getCenter(stack_serial_num);
        loop_closure_data.target_submap_serial_num = stack_serial_num;
        return true;
    }else{
        return false;
    }
}

Eigen::Matrix3d LoopClosureDetection::getRotationMatrix(const double& theta)
{
    Eigen::Matrix3d T;
    double c = cos(theta);
    double s = sin(theta);

    T << c,-s, 0,
         s, c, 0,
         0, 0, 1;

    return T;
}

void LoopClosureDetection::transPoints(const Eigen::Matrix3d& poseMat,
                                       const PCLScanCloudPtr& scan_cloud,
                                       std::vector<Eigen::Vector2i>& pts_tranformed,
                                       const double& resolution)
{
    int pointsNum = scan_cloud->points.size();
    pts_tranformed.reserve(pointsNum);

    for (unsigned int i = 0; i < pointsNum; i++)
    {
        Eigen::Vector2d tmp_pt = transPoint(scan_cloud->points[i], poseMat);
        int cell_x = floor((tmp_pt(0)) / resolution);
        int cell_y = floor((tmp_pt(1)) / resolution);
        Eigen::Vector2i tmp_pt_(cell_x,cell_y);      
        pts_tranformed.push_back(tmp_pt_);
    }
}

Eigen::Vector2d LoopClosureDetection::transPoint(const pcl::PointXYZ& pt,const Eigen::Matrix3d& T)
{
    Eigen::Vector3d tmp_pt(pt.x, pt.y, 1);
    tmp_pt = T * tmp_pt;
    return Eigen::Vector2d(tmp_pt(0), tmp_pt(1));
}

void LoopClosureDetection::computeHighestLevelScore(std::priority_queue<PoseNode>& score_queue,
                                              const OccGridStackPtr& map_stack,
                                              const std::vector<Eigen::Vector2i>& pts_tranformed)
{
    int window_size = 1 << highest_level;
    nav_msgs::OccupancyGridPtr map = (*map_stack)[highest_level];

    for(unsigned int i = 0; i < map->info.height; i+=window_size)
    {
        for(unsigned int j = 0; j < map->info.width; j+=window_size)
        {
            double score = calculateScore(pts_tranformed,map,j,i);
            PoseNode pt(highest_level,score,j,i);
            score_queue.push(pt);
        }
    }   
}

double LoopClosureDetection::calculateScore(const std::vector<Eigen::Vector2i>& points,
                                            const nav_msgs::OccupancyGridPtr& map,const int& x,const int& y)
{
    int width = map->info.width;
    int height = map->info.height;
    int points_num = points.size();

    if(x < 0 ||  x >= width || y < 0 ||  y >= height)
        return -1;

    int index = x + y * width;
    // // ignore unknown space
    if(map->data[index] == 50)
        return -1;

    double total_score = 0;

    for(unsigned int i = 0;i < points_num;i++)
    {
        
        int x_ = points[i](0) + x;
        int y_ = points[i](1) + y;
        double score;

        if(x_ < 0 ||  x_ >= width || y_ < 0 ||  y_ >= height)
            score = 50;
        else{
            int index_ = x_ + y_ * width; 
            score = (double)map->data[index_];
        }

        total_score += score/100;
    }

    if(points_num > 0)
        return total_score/points_num;
    else
        return 0;
}

// main procedure
bool LoopClosureDetection::compute(std::priority_queue<PoseNode>& score_queue,const OccGridStackPtr& map_stack,
                    const std::vector<Eigen::Vector2i>& points, double& max_score, Eigen::Vector2i& max_score_pose)
{
    bool get_higher_score = false;

    while(!score_queue.empty())
    {
        if(score_queue.top().score >= max_score)
        {
            int x = score_queue.top().x;
            int y = score_queue.top().y;
            int offset = 1 << (score_queue.top().level - 1);

            int x_[2],y_[2];
            x_[0] = x; 
            x_[1] = x + offset;
            y_[0] = y; 
            y_[1] = y + offset;

            int current_level = score_queue.top().level;
            double parrent_score = score_queue.top().score; 
            score_queue.pop();

            for(unsigned int i = 0;i < 2;i++)
            {
                for(unsigned int j = 0;j < 2;j++)
                {
                    nav_msgs::OccupancyGridPtr map = (*map_stack)[current_level-1];
                    double score = calculateScore(points,map,x_[i],y_[j]);

                    // over map, ignore
                    if(score < 0)
                        continue;

                    
                    if(score > max_score){
                        if(current_level == 1)
                        {
                            get_higher_score = true;
                            max_score = score;
                            max_score_pose(0) = x_[i];
                            max_score_pose(1) = y_[j];
                            
                        }else{
                            PoseNode pt(current_level -1,score,x_[i],y_[j]);
                            score_queue.push(pt);
                        }
                    }
                }
            }
        }else{
            score_queue.pop();
        }
    }

    return get_higher_score;
}