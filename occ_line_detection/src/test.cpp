#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include "mars_srvs/DetectLine.h"

std::shared_ptr<ros::NodeHandle> nh;
ros::Publisher marker_pub;
ros::Publisher serial_nums_pub;

void pubLines(const hough_line_msgs::Lines& lines)
{
    visualization_msgs::Marker lines_marker;

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

    visualization_msgs::Marker num_marker;
    visualization_msgs::MarkerArray nums_marker;

    num_marker.header.frame_id = "map";
    num_marker.header.stamp = ros::Time();
    num_marker.ns = "nums";
    num_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    num_marker.action = visualization_msgs::Marker::ADD;
    num_marker.id = 0;
    num_marker.scale.z = 0.7;
    num_marker.color.r = 0.0;
    num_marker.color.g = 1.0;
    num_marker.color.b = 0.0;
    num_marker.color.a = 1.0;



    double map_resolution = lines.map_info.resolution; 
    double h = (double)lines.map_info.height * map_resolution;
    double w = (double)lines.map_info.width * map_resolution;
    double origin_x = lines.map_info.origin.position.x;
    double origin_y = lines.map_info.origin.position.y;

    for(unsigned int i = 0; i < lines.lines.size();i++)
    {
        
        geometry_msgs::Point p1;
        geometry_msgs::Point p2;
        geometry_msgs::Point mid;

        double r = lines.lines[i].rho; 
        double angle = lines.lines[i].angle;

        ROS_INFO_STREAM(i+1<<" "<<"r: "<<r<<" a: "<<angle<<" score "<<lines.lines[i].score);

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

        mid.x = (p1.x + p2.x)/2;
        mid.y = (p1.y + p2.y)/2;
        num_marker.text = std::to_string(i+1);
        num_marker.pose.position = mid;
        num_marker.id = i;
        nums_marker.markers.push_back(num_marker);

        lines_marker.points.push_back(p1);
        lines_marker.points.push_back(p2);
    }


    marker_pub.publish(lines_marker);
    serial_nums_pub.publish(nums_marker);
}

void mapCallBack(const nav_msgs::OccupancyGridConstPtr& map)
{   
    ros::ServiceClient client = nh->serviceClient<mars_srvs::DetectLine>("/detect_line");
    mars_srvs::DetectLine srv;
    srv.request.map = *map;

    if(!client.call(srv))
        return;

    pubLines(srv.response.lines);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_occ_line_node");

    nh = std::make_shared<ros::NodeHandle>("~");
    marker_pub = nh->advertise<visualization_msgs::Marker>("visualization_marker", 10,true);
    serial_nums_pub = nh->advertise<visualization_msgs::MarkerArray>("serial_num_marker", 10,true);

    ros::Subscriber sub = nh->subscribe("/map",1,mapCallBack);

    ros::spin();

    return (0);
}