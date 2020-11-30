#include "map_manager.h"

MapManager::MapManager(ros::NodeHandle& nh_):nh(nh_),map_received(false)
{
    initLinesMarker();

    map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map",1,true); 
    lines_marker_pub = nh.advertise<visualization_msgs::MarkerArray>("lines", 1 ,true);
}

void MapManager::initLinesMarker()
{
    lines_marker.header.frame_id = "map";
    lines_marker.header.stamp = ros::Time();
    lines_marker.ns = "lines";
    lines_marker.type = visualization_msgs::Marker::LINE_LIST;
    lines_marker.action = visualization_msgs::Marker::ADD;
    lines_marker.id = 0;
    lines_marker.scale.x = 0.04;
    lines_marker.pose.orientation.w = 1.0;
    lines_marker.color.r = 1.0;
    lines_marker.color.g = 0.0;
    lines_marker.color.b = 0.0;
    lines_marker.color.a = 1.0;
}

bool MapManager::loadMap(const std::string& file_path)
{
    std::ifstream file(file_path);

    if(!file.is_open())
    {
        ROS_ERROR_STREAM("Fail to open file !!");
        return false;
    }else{
        ROS_INFO_STREAM("Open "<< file_path);
    }

    map_file_path = file_path;

    getSstream(file) >> submap_num;
    getSstream(file) >> resolution;

    submaps.clear();
    submaps.reserve(submap_num);
    submaps_pub_array.clear();
    submaps_pub_array.resize(submap_num);

    for(unsigned int i = 0;i < submap_num;i++){
        submaps.push_back(readMap(file));
        submaps_pub_array[i] = nh.advertise<nav_msgs::OccupancyGrid>("submap" + std::to_string(i),1,true);
        submaps_pub_array[i].publish(submaps[i]);
    }

    map = readMap(file);
    map_pub.publish(map);

    file.close();
    ROS_INFO_STREAM("Success to read db file !!");
    map_received = true;

    return true;
}

std::stringstream MapManager::getSstream(std::ifstream& file)
{
    std::string inputLine; 
    std::stringstream ss;

    getline(file,inputLine);
    ss.str(inputLine);

    return ss;
}

nav_msgs::OccupancyGridPtr MapManager::readMap(std::ifstream& file)
{
    nav_msgs::OccupancyGridPtr new_map(new nav_msgs::OccupancyGrid);

    new_map->header.frame_id = "map";
    new_map->header.stamp = ros::Time();
    new_map->info.resolution = resolution;
    getSstream(file) >> new_map->info.width >> new_map->info.height;
    getSstream(file) >> new_map->info.origin.position.x 
                     >> new_map->info.origin.position.y
                     >> new_map->info.origin.position.z
                     >> new_map->info.origin.orientation.x 
                     >> new_map->info.origin.orientation.y
                     >> new_map->info.origin.orientation.z
                     >> new_map->info.origin.orientation.w;   
    int map_size = new_map->info.width * new_map->info.height;
    new_map->data.reserve(map_size);
    std::string inputLine; 
    getline(file,inputLine);

    for(unsigned int i = 0;i < map_size; i++){
        unsigned char temp = inputLine[i];
        int temp_ = temp;
        temp_ = (temp_ - 40)/2;
        if(temp_ > 100)
            temp_ = 100;
        if(temp_ < 0)
            temp_ = 0;
        new_map->data.push_back(temp_);
    }

    return new_map;
}

bool MapManager::detectLine()
{
    if(!map_received){
        ROS_ERROR_STREAM("Please load map first !!");
        return false;
    }

    submap_lines_array.clear();

    for(unsigned int i = 0;i < submap_num;i++)
    {
        ros::ServiceClient client = nh.serviceClient<mars_srvs::DetectLine>("/detect_line");

        mars_srvs::DetectLine srv;
        srv.request.map = *(submaps[i]);

        if(!client.call(srv)){
            ROS_ERROR_STREAM("Fail to detect line !!");
            return false;
        }

        submap_lines_array.push_back(srv.response.lines);
    }

    showLines();
    saveLines();

    ROS_INFO_STREAM("Success to detect lines on submaps ");
    return true;
}
void MapManager::showLines()
{
    double d = resolution/2;
    lines_marker_array.markers.clear();

    for(unsigned int i = 0;i < submap_num;i++)
    {
        lines_marker.points.clear();
        int team = i%10;
        lines_marker.ns = std::to_string(team);
        lines_marker.id = i;

        switch (team%5)
        {
        case 0:
             lines_marker.color = yellow();
            break;
        case 1:
             lines_marker.color = blue();
            break;
        case 2:
             lines_marker.color = green();
            break;
        case 3:
             lines_marker.color = orange();
            break;
        case 4:
             lines_marker.color = purple();
            break;
        default:
            break;
        }

        double h = (double)submaps[i]->info.height * resolution;
        double w = (double)submaps[i]->info.width * resolution;
        tf::Transform transform_to_map;
        tf::poseMsgToTF(submaps[i]->info.origin,transform_to_map);

        for(unsigned int j = 0; j < submap_lines_array[i].lines.size();j++)
        {
            hough_line_msgs::Line line = submap_lines_array[i].lines[j];
            double r = line.rho; 
            double angle = line.angle;
            tf::Point p1,p2;
            geometry_msgs::Point p1_,p2_;
            
            p1.setY(0.0);
            p2.setY(h);

            if(angle == 0.0)
            {
                p1.setX(fabs(r));
                p2.setX(fabs(r));
                
            }else{
                double b = r/sin(angle/180*M_PI);
                double a = -1/tan(angle/180*M_PI);

                p1.setX(-b/a) ;
                p2.setX((p2.y() - b)/a) ;

                if(p1.x() < 0)
                {
                    p1.setX(0);
                    p1.setY(b) ;
                }else if(p1.x() > w)
                {
                    p1.setX(w);
                    p1.setY(a*w+b);
                }

                if(p2.x() < 0)
                {
                    p2.setX(0);
                    p2.setY(b);
                }else if(p2.x() > w)
                {
                    p2.setX(w);
                    p2.setY(a*w+b);
                }
            }

            p1.setX(p1.x() + d); 
            p1.setY(p1.y() + d); 
            p2.setX(p2.x() + d); 
            p2.setY(p2.y() + d); 

            p1 =  transform_to_map*p1;
            p2 =  transform_to_map*p2;

            tf::pointTFToMsg(p1,p1_);
            tf::pointTFToMsg(p2,p2_);

            lines_marker.points.push_back(p1_);  
            lines_marker.points.push_back(p2_);

        }

        lines_marker_array.markers.push_back(lines_marker);
    }

    lines_marker_pub.publish(lines_marker_array);
}

void MapManager::saveLines()
{
    std::string file_path = map_file_path;
    file_path.resize(file_path.size()-2);
    file_path += "lines"; 

    std::ofstream file;
    file.open(file_path);

    if(!file.is_open())
    {
        ROS_ERROR_STREAM("Fail to open " << file_path);
        return;
    }else{
        ROS_INFO_STREAM("Success to open "<< file_path);
    }

    file << submap_num << "\n";
    for(unsigned int i = 0;i < submap_num;i++)
    {
        file << submap_lines_array[i].lines.size() << "\n";
        
        for(unsigned int j = 0;j < submap_lines_array[i].lines.size();j++)
        {
            file << submap_lines_array[i].lines[j].angle << " "
                 << submap_lines_array[i].lines[j].rho << " "
                 << submap_lines_array[i].lines[j].score << "\n";
        }
        
    }


    file.close();
}

std_msgs::ColorRGBA MapManager::green()
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;color.r = 0.0;color.g = 1.0;color.b = 0.0;
    return color;
}

std_msgs::ColorRGBA MapManager::blue()
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;color.r = 0.0;color.g = 0.0;color.b = 1.0;
    return color;
}

std_msgs::ColorRGBA MapManager::yellow()
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;color.r = 1.0;color.g = 0.843;color.b = 0.0;
    return color;
}

std_msgs::ColorRGBA MapManager::orange()
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;color.r = 1;color.g = 0.5;color.b = 0.0;
    return color;
}

std_msgs::ColorRGBA MapManager::purple()
{
    std_msgs::ColorRGBA color;
    color.a = 1.0;color.r = 0.6;color.g = 0.2;color.b = 0.98;
    return color;
}

