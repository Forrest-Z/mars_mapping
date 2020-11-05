#include "mars_db_comparator/db_comparator_node.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "db_comparator_node");

    // main process process
    DBComparator db_comparator;

    ros::spin();

    return (0);
}