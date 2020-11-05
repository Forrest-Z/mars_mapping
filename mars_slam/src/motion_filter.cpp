#include "mars_slam/motion_filter.h"

MotionFilter::MotionFilter(ros::NodeHandle& nh_):nh(nh_)
{
    nh.param<double>("update_min_dis", update_min_dis, 0.1);
    nh.param<double>("update_min_theta", update_min_theta, 0.1);
    nh.param<double>("update_min_time", update_min_time, 0.1);

    queue_max_size = 200;
}

bool MotionFilter::checkUpdate(const tf::Pose& new_pose,const ros::Time& t)
{
    static bool first = true;
    bool update = false;

    if(first)
    {
        first = false;
        last_update_pose = new_pose;
        last_update_time = t;
        update = true;
    }else{
        if(dist(new_pose,last_update_pose) >= update_min_dis ||
           dtheta(new_pose,last_update_pose) >= update_min_theta ||
           dt(t,last_update_time) >= update_min_time)
        {
            last_update_pose = new_pose;
            last_update_time = t;
            update = true;
        }    
    }

    if(update){
        ROS_DEBUG_STREAM("Update map !!");
        update_queue.push_back(true);
    }else{
        ROS_DEBUG_STREAM("Not to update map !!");
        update_queue.push_back(false);
    }

    if(update_queue.size() == queue_max_size)
    {
        int count = 0;
        for(unsigned int i = 0;i < queue_max_size;i++)
            if(update_queue[i])
                count++;

        double rate = (double)count/(double)queue_max_size;
        double percentage = rate*100;    

        ROS_INFO_STREAM("Update rate from motion filter: " << percentage << "%");

        update_queue.clear();
    }

    return update;
}

double MotionFilter::dist(const tf::Pose& a,const tf::Pose& b)
{
    double dx = a.getOrigin().x() - b.getOrigin().x();
    double dy = a.getOrigin().y() - b.getOrigin().y();
    return sqrt(pow(dx,2) + pow(dy,2));
}

double MotionFilter::dtheta(const tf::Pose& a,const tf::Pose& b)
{
    tf::Transform dT = a.inverse()*b;
    return fabs(tf::getYaw(dT.getRotation()));
}

double MotionFilter::dt(const ros::Time& t1,const ros::Time& t2)
{
    return fabs((t2 -t1).toSec());
}
