#include "scan_line_detector/scan_line_detector_node.h"

ScanLineDetector::ScanLineDetector()
{
    nh = std::make_shared<ros::NodeHandle>("~");

    nh->param<std::string>("scan_topic", scan_topic, "/scan");

    scan_sub = nh->subscribe(scan_topic,1,&ScanLineDetector::laserScanCallBack,this);
    marker_pub = nh->advertise<visualization_msgs::Marker>("lines", 1,true);

    map_manager = std::make_unique<MapManager>(*nh);

    initLinesMarker();
}

void ScanLineDetector::initLinesMarker()
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

void ScanLineDetector::laserScanCallBack(const sensor_msgs::LaserScanConstPtr& msg)
{
    static bool first = true;

    // init laserscan processor
    if(first)
    {
        first = false;
        laserscan_processor = std::make_unique<LaserscanProcessor>(*nh,msg);
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr scan_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    laserscan_processor->processLaserscan(msg,scan_cloud); 

    nav_msgs::OccupancyGridPtr map;
    map_manager->transferPointCloudToOccMap(scan_cloud,map);
    callDetectLine(map);


    scan_sub.shutdown();
}

void ScanLineDetector::callDetectLine(const nav_msgs::OccupancyGridPtr& map)
{
    ros::ServiceClient client = nh->serviceClient<mars_srvs::DetectLine>("/detect_line");
    mars_srvs::DetectLine srv;
    srv.request.map = *map;

    if(!client.call(srv)){
        ROS_ERROR_STREAM("Fail to detect line !!");
    }else{
        ROS_INFO_STREAM("Success to detect line");
        pubLines(srv.response.lines);
    }
}

void ScanLineDetector::pubLines(const hough_line_msgs::Lines& lines)
{
    double map_resolution = lines.map_info.resolution; 
    double h = (double)lines.map_info.height * map_resolution;
    double w = (double)lines.map_info.width * map_resolution;
    double origin_x = lines.map_info.origin.position.x;
    double origin_y = lines.map_info.origin.position.y;

    for(unsigned int i = 0; i < lines.lines.size();i++)
    {
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;

        double r = lines.lines[i].rho; 
        double angle = lines.lines[i].angle;

        p1.y = 0;
        p2.y = h;

        if(angle == 0.0)
        {
            p1.x = fabs(r);
            p2.x = p1.x;
            
        }else{
            double b = r/sin(angle/180*M_PI);
            double a = -1/tan(angle/180*M_PI);

            p1.x = -b/a ;
            p2.x = (p2.y - b)/a ;

            if(p1.x < 0)
            {
                p1.x = 0;
                p1.y = b ;
            }else if(p1.x > w)
            {
                p1.x = w;
                p1.y = a*w+b;
            }

            if(p2.x < 0)
            {
                p2.x = 0;
                p2.y = b;
            }else if(p2.x > w)
            {
                p2.x = w;
                p2.y = a*w+b;
            }

        }

        double d = map_resolution/2;
        
        p1.x += origin_x + d;
        p1.y += origin_y + d;
        p2.x += origin_x + d;
        p2.y += origin_y + d; 

        lines_marker.points.push_back(p1);
        lines_marker.points.push_back(p2);
    }


    marker_pub.publish(lines_marker);
}