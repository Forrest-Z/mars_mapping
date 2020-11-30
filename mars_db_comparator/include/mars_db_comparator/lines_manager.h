#ifndef LINES_MANAGER_H
#define LINES_MANAGER_H

#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <hough_line_msgs/Lines.h>
#include "mars_db_comparator/lines_relation.h"

class LinesManager
{
public:
    explicit LinesManager(){}
    explicit LinesManager(ros::NodeHandle& nh_);
    LinesManager(const LinesManager&) = delete;
    ~LinesManager(){}
    void getLineRelation(const hough_line_msgs::Lines& lines,LinesRelationArray& lines_relation_array);
    std::vector<int>& compareLinesRelationsWithDatabase(const LinesRelationArray& scan_lines_relation);

private:
    void loadLinesFile(const std::string& file_path);
    std::stringstream getSstream(std::ifstream& file);
    double angleScore(const LinesRelation& a,const LinesRelation& b);
    double distScore(const LinesRelation& a,const LinesRelation& b);
    bool checkParallel(const LinesRelation& a,const LinesRelation& b);

    ros::NodeHandle nh;

    std::string lines_file_path;

    // param
    double parallel_threshold,sigma_theta,sigma_dist;
    int rank_size;

    int submap_num;
    std::vector<hough_line_msgs::Lines> submap_lines_array;
    std::vector<LinesRelationArray> submap_lines_relations_array;

    std::vector<int> rank_num_array;
    std::vector<double> rank_score_array;
};

#endif