#include "scan_line_detector/scan_line_detector_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "occ_line_detector");

    auto scan_line_detector_node = std::make_unique<ScanLineDetector>();

    ros::spin();

    return (0);
}