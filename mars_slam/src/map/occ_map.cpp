#include "mars_slam/map/occ_map.h"

OccMap::OccMap(const int& initial_map_size,
               const double& resolution_,
               const tf::Pose& center_,
               const int& serial_num_):
init(true),
log_free(-1),
log_occ(2),
log_max(200),
log_confirm_occ(120),
log_unknown(100),
log_confirm_free(80),
log_min(0),
map_expand_speed_by_pixels(200),
update(true),
create_transformed_map(false)
{
    initMap(initial_map_size,resolution_,center_,serial_num_);
}

void OccMap::initMap(const int& initial_map_size,
                    const double& resolution_,
                    const tf::Pose& center_,
                    const int& serial_num_,
                    bool new_)
{
    map_info = std::make_shared<MapInfo>();

    serial_num = serial_num_,
    resolution = resolution_;
    origin_center = transformed_center = center_;
    width = initial_map_size;
    height = initial_map_size;

    map_size = width*height;
    origin_x = ((double)floor(origin_center.getOrigin().x()/resolution))*resolution;
    origin_y = ((double)floor(origin_center.getOrigin().y()/resolution))*resolution;
    origin_x -= (double)(width/2) * resolution;
    origin_y -= (double)(height/2) * resolution;
    map_origin.setOrigin(tf::Vector3(origin_x,origin_y,0.0));
    map_origin.setRotation(tf::Quaternion::getIdentity());

    setMapInfo();
    updateMinMax();
    update_min_x = origin_center.getOrigin().x();
    update_max_x = origin_center.getOrigin().x();
    update_min_y = origin_center.getOrigin().y();
    update_max_y = origin_center.getOrigin().y(); 

    if(!new_)
    {
        deleteMap();
        init = true;
    }

    log_map = new unsigned char[map_size];

    // #pragma omp parallel for
    for(unsigned int i = 0; i < map_size; i++)
        log_map[i] = log_unknown;
}


void OccMap::setMapInfo()
{
    geometry_msgs::Point origin;
    origin.x = origin_x;
    origin.y = origin_y;
    map_info->set(resolution,width,height,origin);
}

void OccMap::updateMinMax()
{
    min_x = origin_x;
    min_y = origin_y;
    max_x = min_x + (double)width*resolution;
    max_y = min_y + (double)height*resolution;
}

void OccMap::addNewScanData(const PCLScanCloudPtr& scan_cloud,const tf::Pose& robot_pose)
{
    ros::Time t1 = ros::Time::now();

    int points_num = scan_cloud->points.size();
    double robot_pose_x = robot_pose.getOrigin().x();
    double robot_pose_y = robot_pose.getOrigin().y();

    int robot_index_x = floor((robot_pose_x - origin_x) / resolution);
    int robot_index_y = floor((robot_pose_y - origin_y) / resolution);

    update_min_x = update_max_x = robot_pose_x;
    update_min_y = update_max_y = robot_pose_y;

    Eigen::Matrix3d pose_matrix = toMatrix(robot_pose);

    LaserScanPtr new_scan = std::make_shared<LaserScan>();
    LaserScanPtr new_scan_global = std::make_shared<LaserScan>();
    
    new_scan->reserve(points_num);
    new_scan_global->reserve(points_num);

    // transform map to global frame
    for (unsigned int i = 0; i < points_num; i++)
    {
        Eigen::Vector2d pt(scan_cloud->points[i].x,scan_cloud->points[i].y);
        Eigen::Vector2d pt_global = transPointToGlobalFrame(pt, pose_matrix);
        if(pt_global[0] < update_min_x)update_min_x = pt_global[0];
        if(pt_global[0] > update_max_x)update_max_x = pt_global[0];       
        if(pt_global[1] < update_min_y)update_min_y = pt_global[1];
        if(pt_global[1] > update_max_y)update_max_y = pt_global[1];
        new_scan->push_back(pt);   
        new_scan_global->push_back(pt_global);   
    }

    scan_poses.push_back(robot_pose);
    scan_data.push_back(new_scan);

    update_mutex[0].lock();
    update_mutex[1].lock();
    update_mutex[2].lock();

    // check whether to expand map
    checkMapSize();

    robot_index_x = floor((robot_pose_x - origin_x) / resolution);
    robot_index_y = floor((robot_pose_y - origin_y) / resolution);
    int robot_index = robot_index_x + robot_index_y * width;

    // update log map
    for (unsigned short i = 0; i < new_scan_global->size(); i++)
    {
        int cell_x = floor(((*new_scan_global)[i][0]-origin_x) / resolution);
        int cell_y = floor(((*new_scan_global)[i][1]-origin_y) / resolution);
        int occ_index = cell_x + cell_y * width;
        std::vector<Eigen::Vector2i> grid_cell;

        traceLine(robot_index_x , robot_index_y , cell_x, cell_y, grid_cell);
        
        // #pragma omp parallel for 
        for(unsigned short j = 1 ;j < grid_cell.size()-1;j++){ 
            int x_grid = grid_cell[j][0];
            int y_grid = grid_cell[j][1];
            int index = x_grid + y_grid * width;

            int tmp = log_map[index] + log_free;
            if(tmp > log_min)
                log_map[index] = tmp;
            else      
                log_map[index] = log_min;
        }

        int tmp;
        tmp = log_map[robot_index] + log_free;
        if(tmp > log_min)
            log_map[robot_index] = tmp;
        else      
            log_map[robot_index] = log_min;
        tmp = log_map[occ_index] + log_occ;
        if(tmp < log_max)
            log_map[occ_index] = tmp;
        else      
            log_map[occ_index] = log_max;
    }

    update_mutex[2].unlock();
    update_mutex[1].unlock();
    update_mutex[0].unlock();
    update = true;

    ros::Time t2 = ros::Time::now();
    ROS_DEBUG_STREAM("Time for updating log map : "<<(t2-t1));

}

Eigen::Matrix3d OccMap::toMatrix(const tf::Pose& robot_pose)
{
    double theta = tf::getYaw(robot_pose.getRotation());

    Eigen::Matrix3d T;
    T << cos(theta), -sin(theta), robot_pose.getOrigin().x(),
         sin(theta), cos(theta), robot_pose.getOrigin().y(),
         0, 0, 1;

    return T;
}

Eigen::Vector2d OccMap::transPointToGlobalFrame(const Eigen::Vector2d& pt,const Eigen::Matrix3d& T)
{
    Eigen::Vector3d tmp_pt(pt(0), pt(1), 1);
    tmp_pt = T * tmp_pt;
    return Eigen::Vector2d(tmp_pt(0), tmp_pt(1));
}

void OccMap::checkMapSize()
{
    bool update_ = false;
    double min_x_new = min_x;
    double min_y_new = min_y;
    double max_x_new = max_x;
    double max_y_new = max_y;

    if(update_min_x <= min_x+1){
        update_ = true;
        min_x_new = update_min_x - resolution*map_expand_speed_by_pixels;
    }
    if(update_min_y <= min_y+1){
        update_ = true;
        min_y_new = update_min_y - resolution*map_expand_speed_by_pixels;
    }
    if(update_max_x >= max_x-1){
        update_ = true;
        max_x_new = update_max_x + resolution*map_expand_speed_by_pixels;
    }
    if(update_max_y >= max_y-1){
        update_ = true;
        max_y_new = update_max_y + resolution*map_expand_speed_by_pixels;
    }

    if(update_)      
        expandMap(min_x_new,min_y_new,max_x_new,max_y_new);
}

void OccMap::expandMap(const double& min_x_new,
                       const double& min_y_new,
                       const double& max_x_new,
                       const double& max_y_new)
{
    int width_new = ceil((max_x_new - min_x_new)/resolution)+1;
    int height_new = ceil((max_y_new - min_y_new)/resolution)+1;
    int map_size_new = width_new * height_new;

    int start_x = ceil((origin_x - min_x_new)/resolution);
    int start_y = ceil((origin_y - min_y_new)/resolution);

    double origin_x_new = origin_x - (double)start_x*resolution;
    double origin_y_new = origin_y - (double)start_y*resolution;

    auto log_map_new = new unsigned char[map_size_new]; 

    // #pragma omp parallel for
    for (unsigned int i = 0; i < height_new; i++)
    {
        for (int j = 0; j < width_new; j++)
        {
            int index_new = j + width_new * i;
            if(i >= start_y && i < start_y + height && j >= start_x && j < start_x + width)
            {
                int index_old = (j - start_x) + width * (i - start_y);
                log_map_new[index_new] = log_map[index_old];
            }else{
                log_map_new[index_new] = log_unknown;
            }
        }
    }

    // set map info and init map
    width = width_new;
    height = height_new;
    map_size = map_size_new;
    origin_x = origin_x_new;
    origin_y = origin_y_new;
    map_origin.setOrigin(tf::Vector3(origin_x,origin_y,0.0));

    setMapInfo();
    updateMinMax();

    delete [] log_map;
    log_map = log_map_new;
}

void OccMap::deleteMap()
{
    if(init)
    {
        init = false;
        delete [] log_map;
    }
}

void OccMap::traceLine(int x0, int y0, int x1, int y1,
                       std::vector<Eigen::Vector2i>& grid_index_vector)
{
    Eigen::Vector2i tmp_index;

    bool steep = abs(y1 - y0) > abs(x1 - x0);
    if (steep)
    {
        std::swap(x0, y0);
        std::swap(x1, y1);
    }
    if (x0 > x1)
    {
        std::swap(x0, x1);
        std::swap(y0, y1);
    }

    int delta_x = x1 - x0;
    int delta_y = y1 - y0;
    double slope = (double)delta_y/delta_x;

    int y = y0;
    int point_x;
    int point_y;

    grid_index_vector.reserve(x1 - x0 + 1);

    for (int x = x0; x <= x1; x++)
    {
        y = y0+round(slope*(double)(x-x0));
        if (steep)
        {
            point_x = y;
            point_y = x;
        }
        else
        {
            point_x = x;
            point_y = y;
        }
        
        tmp_index[0] = point_x;
        tmp_index[1] = point_y;
        grid_index_vector.push_back(tmp_index);
    }
}

void OccMap::initMapPub(ros::NodeHandle& nh)
{
    if(serial_num >= 0)
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("submap" + std::to_string(serial_num),1,true);
    else
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map",1,true);
}

void OccMap::publishMap()
{
    if(!update || !init)
        return;

    nav_msgs::OccupancyGrid map_msg;

    update_mutex[0].lock();

    map_msg.header.frame_id = "map";
    map_msg.header.stamp = ros::Time();
    map_msg.info.width = width;
    map_msg.info.height = height;
    map_msg.info.resolution = resolution;
    tf::poseTFToMsg(map_origin,map_msg.info.origin);
    // map_msg.info.origin.position.x = origin_x;
    // map_msg.info.origin.position.y = origin_y;
    // map_msg.info.origin.orientation.w = 1;

    map_msg.data.reserve(map_size);
    double r = 100.0/(double)(log_max - log_min);

    for(int i = 0;i < map_size;i++){
        int tmp = round((double)(log_map[i]-log_min) * r);
        if(tmp < 0)
            tmp = 0;
        if(tmp > 100){
            tmp = 100;
        }
        map_msg.data.push_back(tmp);
    }

    map_pub.publish(map_msg);

    update = false;
    update_mutex[0].unlock();
}

void OccMap::combineMap(OccMap& m_)
{
    m_.lockMapUpdate(2);
    this->lockMapUpdate(2);

    this->update_min_x = m_.min_x;
    this->update_max_x = m_.max_x;
    this->update_min_y = m_.min_y;
    this->update_max_y = m_.max_y;

    checkMapSize();

    int start_x = round((m_.origin_x - this->origin_x)/resolution);
    int start_y = round((m_.origin_y - this->origin_y)/resolution);

    for(unsigned int i = 0;i < m_.height;i++)
    {
        for(unsigned int j = 0;j < m_.width;j++)
        {
            int new_index = (j + start_x) + this->width * (i + start_y);
            int index = j + m_.width * i;

            int a = this->log_map[new_index];
            int b = m_.log_map[index];
            if(b != log_unknown)
            {
                if(a == log_unknown)
                {
                    this->log_map[new_index] = b;
                }else{
                    int c = a + b - log_unknown;
                    if(c > log_max)c = log_max;
                    if(c < log_min)c = log_min;
                    this->log_map[new_index] = c;
                }
            }

        }
    }
    this->update = true;

    this->unlockMapUpdate(2);
    m_.unlockMapUpdate(2);
}

void OccMap::copyMap(OccMap& m_)
{
    m_.lockMapUpdate(2);
    this->map_info = m_.map_info;
    this->origin_x = m_.origin_x;
    this->origin_y = m_.origin_y;
    this->map_origin = m_.map_origin;
    this->width = m_.width;
    this->height = m_.height;
    this->map_size = m_.map_size;
    this->min_x = m_.min_x;
    this->min_y = m_.min_y;
    this->max_x = m_.max_x;
    this->max_y = m_.max_y;

    delete [] this->log_map;

    this->log_map = new unsigned char[this->map_size];

    for(unsigned int i = 0;i < this->map_size;i++)
        this->log_map[i] = m_.log_map[i];

    m_.unlockMapUpdate(2);
}

void OccMap::updateMapByOptimizationPoses(const VerticesPtr& first_poses_array,
                                          const VerticesPtr& second_poses_array)
{
    update_mutex[0].lock();
    update_mutex[1].lock();
    update_mutex[2].lock();

    renewPoses(first_poses_array,second_poses_array);
    updateMapOrigin();
    createTransformedMap();
    // renewMap(800);

    update_mutex[2].unlock();
    update_mutex[1].unlock();
    update_mutex[0].unlock();

    // ROS_INFO_STREAM("Finish update submap "<<serial_num);
}

void OccMap::renewPoses(const VerticesPtr& first_poses_array,const VerticesPtr& second_poses_array)
{
    renewFirstPartPoses(first_poses_array);
    renewSecondPartPoses(second_poses_array);
    transformed_center = scan_poses[0];
}

void OccMap::renewFirstPartPoses(const VerticesPtr& first_poses_array)
{
    for(unsigned int i = 0;i < own_scan_num;i++)
    {
        // tf::poseEigenToTF((*first_poses_array)[i]->estimate(),scan_poses[i]);
        poseEigenToTF((*first_poses_array)[i]->estimate().toIsometry(),scan_poses[i]);

    }
}

void OccMap::renewSecondPartPoses(const VerticesPtr& second_poses_array)
{
    for(unsigned int i = own_scan_num;i < scan_poses.size();i++)
    {
        // tf::poseEigenToTF((*second_poses_array)[i - own_scan_num]->estimate(),scan_poses[i]);
        poseEigenToTF((*second_poses_array)[i - own_scan_num]->estimate().toIsometry(),scan_poses[i]);
    }    
}

void OccMap::renewMap(const int& initial_map_size)
{
    if(init)
    {
        deleteMap();
        init = true;
    }

    map_info = std::make_shared<MapInfo>();

    
    width = initial_map_size;
    height = initial_map_size;

    map_size = width*height;
    origin_x = ((double)floor(transformed_center.getOrigin().x()/resolution))*resolution;
    origin_y = ((double)floor(transformed_center.getOrigin().y()/resolution))*resolution;
    origin_x -= (double)(width/2) * resolution;
    origin_y -= (double)(height/2) * resolution;
    map_origin.setOrigin(tf::Vector3(origin_x,origin_y,0.0));
    setMapInfo();
    updateMinMax();
    update_min_x = transformed_center.getOrigin().x();
    update_max_x = transformed_center.getOrigin().x();
    update_min_y = transformed_center.getOrigin().y();
    update_max_y = transformed_center.getOrigin().y(); 

    log_map = new unsigned char[map_size];

    // #pragma omp parallel for
    for(unsigned int i = 0; i < map_size; i++)
        log_map[i] = log_unknown;
    
    for(unsigned int i = 0; i < scan_poses.size(); i++)
        addScanData(scan_data[i],scan_poses[i]);
}

void OccMap::addScanData(const LaserScanPtr& scan,const tf::Pose& robot_pose)
{
    int points_num = scan->size();
    double robot_pose_x = robot_pose.getOrigin().x();
    double robot_pose_y = robot_pose.getOrigin().y();

    int robot_index_x = floor((robot_pose_x - origin_x) / resolution);
    int robot_index_y = floor((robot_pose_y - origin_y) / resolution);

    update_min_x = update_max_x = robot_pose_x;
    update_min_y = update_max_y = robot_pose_y;

    Eigen::Matrix3d pose_matrix = toMatrix(robot_pose);

    LaserScanPtr scan_global = std::make_shared<LaserScan>();
    
    scan_global->reserve(points_num);

    // transform map to global frame
    for (unsigned int i = 0; i < points_num; i++)
    {
        Eigen::Vector2d pt_global = transPointToGlobalFrame((*scan)[i], pose_matrix);
        if(pt_global[0] < update_min_x)update_min_x = pt_global[0];
        if(pt_global[0] > update_max_x)update_max_x = pt_global[0];       
        if(pt_global[1] < update_min_y)update_min_y = pt_global[1];
        if(pt_global[1] > update_max_y)update_max_y = pt_global[1];  
        scan_global->push_back(pt_global);   
    }

    // check whether to expand map
    checkMapSize();

    robot_index_x = floor((robot_pose_x - origin_x) / resolution);
    robot_index_y = floor((robot_pose_y - origin_y) / resolution);
    int robot_index = robot_index_x + robot_index_y * width;

    // update log map
    for (unsigned short i = 0; i < scan_global->size(); i++)
    {
        int cell_x = floor(((*scan_global)[i][0]-origin_x) / resolution);
        int cell_y = floor(((*scan_global)[i][1]-origin_y) / resolution);
        int occ_index = cell_x + cell_y * width;
        std::vector<Eigen::Vector2i> grid_cell;

        traceLine(robot_index_x , robot_index_y , cell_x, cell_y, grid_cell);
        
        // #pragma omp parallel for 
        for(unsigned short j = 1 ;j < grid_cell.size()-1;j++){ 
            int x_grid = grid_cell[j][0];
            int y_grid = grid_cell[j][1];
            int index = x_grid + y_grid * width;

            int tmp = log_map[index] + log_free;
            if(tmp > log_min)
                log_map[index] = tmp;
            else      
                log_map[index] = log_min;
        }

        int tmp;
        tmp = log_map[robot_index] + log_free;
        if(tmp > log_min)
            log_map[robot_index] = tmp;
        else      
            log_map[robot_index] = log_min;
        tmp = log_map[occ_index] + log_occ;
        if(tmp < log_max)
            log_map[occ_index] = tmp;
        else      
            log_map[occ_index] = log_max;
    }

    update = true;
}

void OccMap::copyOriginalMapToHalfMap()
{
    height_half = height;
    width_half = width;
    map_size_half = map_size;
    origin_x_half = origin_x;
    origin_y_half = origin_y;

    log_map_half = new unsigned char[map_size_half];

    for(unsigned int i = 0;i < map_size_half;i++)
        log_map_half[i] = log_map[i];
}

void OccMap::createTransformedMap()
{
    std::queue<TransformedGrid> grids;
    tf::Transform T = transformed_center * origin_center.inverse();

    double max_x_,max_y_,min_x_,min_y_;
    max_x_ = max_y_ = -1000000;
    min_x_ = min_y_ =  1000000;

    for(unsigned int i = 0;i < height_half;i++)
    {
        for(unsigned int j = 0;j < width_half;j++)
        {
            int index = j + i * width_half;
            if(log_map_half[index] != log_unknown)
            {
                double x = origin_x_half + (double)j * resolution + resolution/2;
                double y = origin_y_half + (double)i * resolution + resolution/2;
                tf::Point origin_pt(x,y,1);
                tf::Point transformed_pt = T * origin_pt;
                double x_transformed = transformed_pt.x();
                double y_transformed = transformed_pt.y();
                TransformedGrid grid;

                grid.score = log_map_half[index] - log_unknown;
                grid.x = x_transformed;
                grid.y = y_transformed;
                
                grids.push(grid);
                
                if(x_transformed > max_x_)max_x_ = x_transformed;
                if(x_transformed < min_x_)min_x_ = x_transformed;
                if(y_transformed > max_y_)max_y_ = y_transformed;
                if(y_transformed < min_y_)min_y_ = y_transformed;

            }
        }
    }

    width_transformed = ceil((max_x_ - min_x_)/resolution)+1;
    height_transformed = ceil((max_y_ - min_y_)/resolution)+1;
    map_size_transformed = width_transformed * height_transformed;

    origin_x_transformed = ((double)floor(min_x_/resolution))*resolution;
    origin_y_transformed = ((double)floor(min_y_/resolution))*resolution;

    if(create_transformed_map)
    {
        delete [] log_map_transformed;
    }else
    {
        create_transformed_map = true;
    }
    
    log_map_transformed = new unsigned char[map_size_transformed]; 

    for(unsigned int i = 0;i < map_size_transformed;i++)
        log_map_transformed[i] = log_unknown;


    while(grids.size() > 0)
    {
        TransformedGrid grid = grids.front();
        grids.pop();

        int x = floor((grid.x - origin_x_transformed)/resolution);
        int y = floor((grid.y - origin_y_transformed)/resolution);

        int index = x + y * width_transformed;

        int temp = log_map_transformed[index];
        temp = temp + grid.score;
        if(temp > log_max)
            temp = log_max;
        if(temp < log_min)
            temp = log_min;
        log_map_transformed[index] = temp;
    }
}

void OccMap::combineMapByTransformedMap(OccMap& m_)
{
    m_.lockMapUpdate(2);
    this->lockMapUpdate(2);

    this->update_min_x = m_.origin_x_transformed;
    this->update_max_x = m_.origin_x_transformed + m_.resolution * (double)m_.width_transformed;
    this->update_min_y = m_.origin_y_transformed;
    this->update_max_y = m_.origin_y_transformed + m_.resolution * (double)m_.height_transformed;

    this->checkMapSize();

    int start_x = round((m_.origin_x_transformed - this->origin_x)/resolution);
    int start_y = round((m_.origin_y_transformed - this->origin_y)/resolution);

    for(unsigned int i = 0;i < m_.height_transformed;i++)
    {
        for(unsigned int j = 0;j < m_.width_transformed;j++)
        {
            int new_index = (j + start_x) + this->width * (i + start_y);
            int index = j + m_.width_transformed * i;

            int a = this->log_map[new_index];
            int b = m_.log_map_transformed[index];
            if(b != log_unknown)
            {
                if(a == log_unknown)
                {
                    this->log_map[new_index] = b;
                }else{
                    int c = a + b - log_unknown;
                    if(c > log_max)c = log_max;
                    if(c < log_min)c = log_min;
                    this->log_map[new_index] = c;
                }
            }

        }
    }

    this->update = true;

    this->unlockMapUpdate(2);
    m_.unlockMapUpdate(2);
}

void OccMap::updateMapOrigin()
{
    tf::Transform T = transformed_center * origin_center.inverse();
    tf::Pose map_origin_temp;
    map_origin_temp.setOrigin(tf::Vector3(origin_x,origin_y,0.0));
    map_origin_temp.setRotation(tf::Quaternion::getIdentity());
    map_origin = T * map_origin_temp;
    update = true;
}

void OccMap::poseEigenToTF(const Eigen::Isometry2d& pose_eigen,tf::Pose& pose_tf)
{
    double x = pose_eigen.matrix()(0,2);
    double y = pose_eigen.matrix()(1,2);
    double theta =  atan2(pose_eigen.matrix()(1,0),pose_eigen.matrix()(0,0));
    tf::Quaternion q;
    q.setRPY(0.0,0.0,theta);

    pose_tf.setOrigin(tf::Vector3(x,y,0.0));
    pose_tf.setRotation(q);
}

void OccMap::narrowMap()
{
    update_mutex[0].lock();
    update_mutex[1].lock();
    update_mutex[2].lock();

    int narrow_up = 0;
    int narrow_down = 0;
    int narrow_left = 0;
    int narrow_right = 0;

    // left
    for(unsigned int i = 0 ; i < width; i++)
    {
        bool narrow = true;
        for(unsigned int j = 0 ; j < height; j++)
        {
            int index = i + j * width;
            if(log_map[index] >= log_confirm_occ || log_map[index] <= log_confirm_free){
                narrow = false;
                break;
            }
        }
        if(narrow)
            narrow_left++;
        else
            break;
    }

    // right 
    for(unsigned int i = width -1 ; i >= 0; i--)
    {
        bool narrow = true;
        for(unsigned int j = 0 ; j < height; j++)
        {
            int index = i + j * width;
            if(log_map[index] >= log_confirm_occ || log_map[index] <= log_confirm_free){
                narrow = false;
                break;
            }
        }
        if(narrow)
            narrow_right++;
        else
            break;
    }

    // down
    for(unsigned int i = 0 ; i < height; i++)
    {
        bool narrow = true;
        for(unsigned int j = 0 ; j < width; j++)
        {
            int index = j + i * width;
            if(log_map[index] >= log_confirm_occ || log_map[index] <= log_confirm_free){
                narrow = false;
                break;
            }
        }
        if(narrow)
            narrow_down++;
        else
            break;
    }

    // up
    for(unsigned int i = height -1; i >= 0; i--)
    {
        bool narrow = true;
        for(unsigned int j = 0 ; j < width ; j++)
        {
            int index = j + i * width;
            if(log_map[index] >= log_confirm_occ || log_map[index] <= log_confirm_free){
                narrow = false;
                break;
            }
        }
        if(narrow)
            narrow_up++;
        else
            break;
    }

    unsigned char* log_map_new;

    int width_new = width - narrow_left - narrow_right;
    int height_new = height - narrow_up - narrow_down;

    tf::Pose map_origin_new;
    tf::Pose temp = tf::Pose::getIdentity();
    temp.setOrigin(tf::Vector3((double)narrow_left * resolution,(double)narrow_down * resolution,0.0));
    map_origin_new = map_origin*temp;

    log_map_new = new unsigned char[width_new*height_new];

    #pragma omp parallel for
    for(unsigned int i = 0; i < height_new; i++)
    {
        for(unsigned int j = 0;j < width_new;j++)
        {
            int index = j + i * width_new;
            int index_old = (j+narrow_left) + (i+narrow_down) * width;

            log_map_new[index] = log_map[index_old];
        }
    }

    width = width_new;
    height = height_new;
    map_size = width_new * height_new;
    map_origin = map_origin_new;

    delete [] log_map;
    log_map = log_map_new;

    update = true;

    update_mutex[2].unlock();
    update_mutex[1].unlock();
    update_mutex[0].unlock();    
}

void OccMap::saveMap(std::ofstream &file)
{
    file << width << " " << height << "\n";
    file << map_origin.getOrigin().x() << " "
         << map_origin.getOrigin().y() << " "
         << map_origin.getOrigin().z() << " "
         << map_origin.getRotation().x() << " "
         << map_origin.getRotation().y() << " "
         << map_origin.getRotation().z() << " "
         << map_origin.getRotation().w() << "\n";

    for(unsigned int i = 0;i < map_size;i++)
    {
        unsigned char a = log_map[i] + 40;
        file << a;
    }
    file << "\n";     

}
