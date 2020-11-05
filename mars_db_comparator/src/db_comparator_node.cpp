#include "mars_db_comparator/db_comparator_node.h"

DBComparator::DBComparator()
{
    nh = std::make_shared<ros::NodeHandle>("~");

    map_manager = std::make_unique<MapManager>(*nh);
    lines_manager = std::make_unique<LinesManager>(*nh);

}