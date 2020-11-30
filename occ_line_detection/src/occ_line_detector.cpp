#include "occ_line_detection/occ_line_detector.h"

OccLineDetector::OccLineDetector()
{
    nh = std::make_shared<ros::NodeHandle>("~");

    nh->param<int>("omp_max_thread_num", omp_max_thread_num, 4);
    nh->param<double>("angle_resolution", angle_resolution, 1);
    nh->param<double>("r_resolution", r_resolution, 0.05);
    nh->param<double>("angle_window_size", angle_window_size, 2.0);
    nh->param<double>("r_window_size", r_window_size, 0.3);
    nh->param<int>("line_threshold", line_threshold, 2000);

    nh->param<bool>("inflate_map", inflate_map, true);
    nh->param<bool>("pub_map", pub_map, true);

    if(pub_map){
        if(inflate_map)
            inflate_map_pub = nh->advertise<nav_msgs::OccupancyGrid>("inflate_map",1,true);
        
        hough_map_pub = nh->advertise<nav_msgs::OccupancyGrid>("hough_map",1,true);
    }

    omp_set_num_threads(omp_max_thread_num);

    set_map_server = nh->advertiseService("/detect_line",&OccLineDetector::lineDetection,this);
}

bool OccLineDetector::lineDetection(mars_srvs::DetectLine::Request& req,
                                    mars_srvs::DetectLine::Response& res)
{
    ROS_INFO_STREAM("Receive map and start to detect line !!");
    ros::Time t1 = ros::Time::now();

    nav_msgs::OccupancyGridPtr map_(new nav_msgs::OccupancyGrid);

    *map_ = req.map;

    if(inflate_map)
        inflateMap(req.map,map_);

    double resolution = map_->info.resolution;
    int width = map_->info.width;
    int height = map_->info.height;
    double w = (double)width * resolution;
    double h = (double)height * resolution;
    
    int r_size = ceil(sqrt(pow(w,2)+pow(h,2))/r_resolution)*2;
    int angle_size = ceil(180.0/angle_resolution);
    
    ROS_INFO_STREAM("angle_size: " << angle_size);

    int map_size = angle_size * r_size;

    double origin_r = -(double)r_size*r_resolution/2;
    double origin_a = 0;

    // std::unique_ptr<unsigned int[]> hough_map = std::make_unique<unsigned int[]>(map_size);
    int* hough_map = new int[map_size]; 

    for(unsigned int i = 0;i < map_size;i++)
        hough_map[i] = 0;

    hough_line_msgs::Lines lines;
    lines.map_info = map_->info;  

    #pragma omp parallel for
    for(int i = 0;i < height;i++)
    {
        for(int j = 0;j < width;j++)
        {
            int index = j + i * width;
            if(i == 0 && j == 0)
                continue;

            if(map_->data[index] >= 60)
            {
                double x_ = (double)j + 0.5;
                double y_ = (double)i + 0.5;

                double phi = atan2(y_,x_);
                double R = sqrt(pow(x_,2) + pow(y_,2)) * resolution;

                // #pragma omp parallel for
                for(int k = 0;k < angle_size; k++)
                {
                    double angle = (double)k *  angle_resolution;
                    double r = R*cos( angle/180.0*M_PI - phi);
                    // int theta_index = (angle-origin_a)/angle_resolution;
                    int theta_index = k;
                    int r_index = (r-origin_r)/r_resolution;
                    int hough_index = r_index*angle_size + theta_index;
                    hough_map[hough_index]++;
                }
            }
            
        }
    }  

    int max = 0;

    for(unsigned int i= 0;i < r_size;i++)
    {
        for (unsigned int j = 0; j < angle_size; j++)
        {
            int hough_index = i*angle_size + j;

            if(hough_map[hough_index] > max)
                max = hough_map[hough_index] ;
           
            if(hough_map[hough_index] > line_threshold)
            {
                
                hough_line_msgs::Line new_line;
                double r = origin_r + (double)i * r_resolution;
                // double angle = origin_a + (double)j * angle_resolution;
                double angle = (double)j * angle_resolution;
                
                new_line.rho = r;
                new_line.angle = angle;
                new_line.score = hough_map[hough_index];

                if(!checkRepeat(new_line,lines))
                    lines.lines.push_back(new_line);
                
            }
        }
    }

    if(pub_map){
        nav_msgs::OccupancyGrid hough_map_;
        hough_map_.header.frame_id = "map";
        hough_map_.info.height = r_size;
        hough_map_.info.width = angle_size;
        hough_map_.info.resolution = 0.05;
        hough_map_.info.origin.position.x = 0;
        hough_map_.info.origin.position.y = 0;
        hough_map_.info.origin.position.z = 0;


        hough_map_.data.resize(map_size);
        for(int i=0;i<map_size;i++)
            hough_map_.data[i] = (100 - ceil((double)hough_map[i]/(double)max*100));

        hough_map_pub.publish(hough_map_);
    }
    delete [] hough_map;

    res.lines = lines;

    ROS_INFO_STREAM("Max: " << max);

    ROS_INFO_STREAM("Lines Num: "<< lines.lines.size());

    ros::Time t2 = ros::Time::now();
    ROS_INFO_STREAM("Finish line detection !! "<<t2-t1);

    return true;
}

bool OccLineDetector::checkRepeat(const hough_line_msgs::Line& new_line,hough_line_msgs::Lines& lines)
{
    for(int i = 0;i < lines.lines.size();i++)
        if((fabs(new_line.rho - lines.lines[i].rho) <= r_window_size ||
            fabs(new_line.rho + lines.lines[i].rho) <= r_window_size) && 
            (fabs(new_line.angle - lines.lines[i].angle) <= angle_window_size || 
             180.0 - fabs(new_line.angle - lines.lines[i].angle) <= angle_window_size))
        {

            if(new_line.score > lines.lines[i].score)
                lines.lines[i] = new_line;
            
            return true;
        }

    return false;
}

void OccLineDetector::inflateMap(const nav_msgs::OccupancyGrid& origin_map,nav_msgs::OccupancyGridPtr& new_map)
{
    #pragma omp parallel for
    for(unsigned int i = 1;i < origin_map.info.height - 1;i++)
    {
        for(unsigned int j = 1;j < origin_map.info.width - 1;j++)
        {
            int index_new = i * origin_map.info.width + j;
            int max = -1;

            for(int k = -1;k <= 1;k++)
            {
                for(int d = -1;d <= 1;d++)
                {
                    int index_old = (i+k) * origin_map.info.width + (j+d);

                    if(origin_map.data[index_old] > max)
                        max = origin_map.data[index_old];
                }
            }

            new_map->data[index_new] = max;
        }
    }

    if(pub_map)
        inflate_map_pub.publish(new_map);
}