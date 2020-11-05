#ifndef DB_COMPARATOR_NODE_H
#define DB_COMPARATOR_NODE_H

#include <ros/ros.h>
#include "mars_db_comparator/map_manager.h"
#include "mars_db_comparator/lines_manager.h"

class DBComparator
{
public:
    explicit DBComparator();
    ~DBComparator(){}
    DBComparator(const DBComparator&) = delete;

private:
    std::shared_ptr<ros::NodeHandle> nh;

    std::unique_ptr<MapManager> map_manager;
    std::unique_ptr<LinesManager> lines_manager;

};


#endif 