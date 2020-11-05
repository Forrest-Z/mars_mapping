#ifndef POSE_EXTRAPOLATOR_H
#define POSE_EXTRAPOLATOR_H

#include <future>
#include <queue>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class PoseExtrapolator
{
public:
    explicit PoseExtrapolator(){}
    explicit PoseExtrapolator(ros::NodeHandle& nh_);
    ~PoseExtrapolator(){}
    PoseExtrapolator(const PoseExtrapolator&) = delete; 
    bool getScanFrameId(){return get_scan_frame_id;}
    void setScanFrame(const std::string& scan_frame_id_){scan_frame_id = scan_frame_id_;get_scan_frame_id = true;}
    bool getInitGuess(const ros::Time& t,tf::Pose& guess);
    void updatePoseArrayAndPubTF(const tf::Pose& new_pose,const ros::Time& t);
    bool openOdomSubThread(){return open_odom_sub_thread;}
    void openOdomSub(); 
    const tf::Transform& getTfToCorrectedPose(){return transform_to_corrected_pose;}
    bool getLastPose(tf::Pose& last_pose);

private:
    bool getTransformToCorrectedPose();
    bool getTransform(const ros::Time& t,tf::Transform& transform,
                      const std::string& base_frame, const std::string& target_frame);
    tf::Transform getDeltaLaserPose(const tf::Transform& robot_on_odom);
    void publishTransform(const ros::Time& t,const tf::Pose& pose);
    bool getRobotOnOdomFromQueue(const ros::Time& t,tf::Transform& transform);
    bool getOdom(tf::Transform& transform,const ros::Time& t);
    void odomCallback(const nav_msgs::OdometryConstPtr& msg);
    void addOdomMsg(const nav_msgs::OdometryConstPtr& msg);


    ros::NodeHandle nh;

    bool use_odom_input;
    bool odom_by_tf;
    std::string odom_topic;
    std::string odom_frame_id;
    std::string robot_frame_id;
    std::string published_frame_id;
    std::string scan_frame_id;
    double transform_tolerance;
    double transform_offset;

    bool get_scan_frame_id;
    bool get_tf_to_corrected_pose;

    tf::Transform transform_to_corrected_pose;
    tf::Transform corrected_laser_on_robot;
    tf::Transform laser_on_robot;
    tf::Transform robot_on_published_frame; 
    tf::Pose last_robot_on_odom;  

    std::vector<tf::Pose> pose_array;
    tf::TransformListener tf_listener;
    tf::TransformBroadcaster tf_broadcaster;

    // odom queue
    bool open_odom_sub_thread;
    ros::Time newest_time;
    boost::mutex odom_queue_mutex;
    std::queue<nav_msgs::Odometry> odom_queue;
    bool init_odom_front;
    nav_msgs::Odometry odom_temp_front;
    double odom_queue_max_size;
    ros::Subscriber odom_sub;

    int start_warn_count;
};

#endif