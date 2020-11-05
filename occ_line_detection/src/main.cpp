#include "occ_line_detection/occ_line_detector.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "occ_line_detector");

    auto occ_line_detector = std::make_unique<OccLineDetector>();

    ros::spin();

    return (0);
}