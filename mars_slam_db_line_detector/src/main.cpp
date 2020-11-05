#include "db_line_detector_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_line_detector_node");

    // main process process
    DBLineDetector db_line_detector_node;

    ros::spin();

    return (0);
}