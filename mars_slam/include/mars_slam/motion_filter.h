#ifndef MOTION_FILTER_H
#define MOTION_FILTER_H

#include <ros/ros.h>
#include <tf/transform_datatypes.h>

class MotionFilter
{
public:
    explicit MotionFilter(){}
    explicit MotionFilter(ros::NodeHandle& nh_);
    MotionFilter(const MotionFilter&) = delete;
    ~MotionFilter(){}
    bool checkUpdate(const tf::Pose& new_pose,const ros::Time& t);
    double dist(const tf::Pose& a,const tf::Pose& b);

private:
    double dtheta(const tf::Pose& a,const tf::Pose& b);
    double dt(const ros::Time& t1,const ros::Time& t2);

    ros::NodeHandle nh;

    // param
    double update_min_dis,update_min_theta,update_min_time;

    tf::Pose last_update_pose;
    ros::Time last_update_time;
    int queue_max_size;
    std::vector<bool> update_queue; 

};

#endif 