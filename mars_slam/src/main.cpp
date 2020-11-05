#include "mars_slam/mars_slam_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "mars_slam_node");

    // main process process
    MarsSlam mars_slam_node;

    ros::spin();

    return (0);
}