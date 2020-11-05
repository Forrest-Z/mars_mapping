#include "mars_slam/pose_extrapolator.h"

PoseExtrapolator::PoseExtrapolator(ros::NodeHandle& nh_):
nh(nh_),
get_tf_to_corrected_pose(false),
get_scan_frame_id(false),
init_odom_front(false),
odom_queue_max_size(2.0),
open_odom_sub_thread(false),
start_warn_count(10)
{
    ROS_INFO_STREAM("Init pose etrapolator !!");

    nh.param<bool>("use_odom_input", use_odom_input, true);
    nh.param<bool>("odom_by_tf", odom_by_tf, true);
    nh.param<std::string>("odom_topic", odom_topic, "odom");
    nh.param<std::string>("odom_frame_id", odom_frame_id, "odom");
    nh.param<std::string>("robot_frame_id", robot_frame_id, "base_link");
    nh.param<std::string>("published_frame_id", published_frame_id, "odom");
    nh.param<double>("transform_tolerance", transform_tolerance, 0.1);
    nh.param<double>("transform_offset", transform_offset, 0.1);

    if(use_odom_input && !odom_by_tf)
        open_odom_sub_thread = true;   
}

bool PoseExtrapolator::getInitGuess(const ros::Time& t,tf::Pose& guess)
{
    if(start_warn_count > 0)
        start_warn_count--;

    if(!get_tf_to_corrected_pose)
    {
        if(!getTransformToCorrectedPose())
            return false;
        else
            get_tf_to_corrected_pose = true;
    }

    if(!getTransform(t,robot_on_published_frame,published_frame_id,robot_frame_id))
        return false;

    if(use_odom_input)
    {
        tf::Pose robot_on_odom;
        if(odom_by_tf)
        {
            if(!getTransform(t,robot_on_odom,odom_frame_id,robot_frame_id))
                return false; 
        }else{
            if(!getRobotOnOdomFromQueue(t,robot_on_odom))
                return false;   
        }

        if(pose_array.size() == 0)
        {
            guess = tf::Pose::getIdentity();
        }else{
            guess = pose_array.back()*getDeltaLaserPose(robot_on_odom);
        }

        last_robot_on_odom = robot_on_odom;
    }else // no use odom
    { 
        if(pose_array.size() == 0)
        {
            guess = tf::Pose::getIdentity();
        }else if(pose_array.size() == 1)
        {
            guess = pose_array.back();
        }else{
            tf::Transform dl = pose_array[pose_array.size()-2].inverse()*pose_array[pose_array.size()-1];

            if(dl.getOrigin().x() > 0.3 || dl.getOrigin().y() > 0.3 || tf::getYaw(dl.getRotation()) > M_PI/6)
            {
                guess = pose_array.back();
            }else{
                guess = pose_array.back()*dl;
            }
        }
    }

    return true;
}

// get transform from laser to robot plane
bool PoseExtrapolator::getTransformToCorrectedPose()
{
    if(!get_scan_frame_id)
        return false;

    if(!getTransform(ros::Time(),laser_on_robot,robot_frame_id,scan_frame_id))
        return false;
    
    corrected_laser_on_robot = laser_on_robot;
    tf::Quaternion q;
    double yaw = tf::getYaw(corrected_laser_on_robot.getRotation());
    q.setRPY(0.0,0.0,yaw);
    corrected_laser_on_robot.setRotation(q);
    transform_to_corrected_pose = corrected_laser_on_robot.inverse()*laser_on_robot;

    return true;
}

bool PoseExtrapolator::getTransform(const ros::Time& t,tf::Transform& transform,
                                    const std::string& base_frame, const std::string& target_frame)                
{
    tf::StampedTransform transform_;
    try{
        tf_listener.waitForTransform(base_frame, target_frame, t, ros::Duration(transform_tolerance));
        tf_listener.lookupTransform(base_frame, target_frame, t, transform_);                         
    }
    catch (tf::TransformException ex){
        if(start_warn_count <= 0)
            ROS_WARN_STREAM(ex.what());
        return false;
    }    
    transform = transform_;
    return true;
}

void PoseExtrapolator::updatePoseArrayAndPubTF(const tf::Pose& new_pose,const ros::Time& t)
{
    pose_array.push_back(new_pose);
    publishTransform(t,new_pose);
}

void PoseExtrapolator::publishTransform(const ros::Time& t,const tf::Pose& pose)
{
    tf::Transform map_to_published_frame = pose*
                                           corrected_laser_on_robot.inverse()*
                                           robot_on_published_frame.inverse();  

    map_to_published_frame.getOrigin().setZ(0); 

    tf_broadcaster.sendTransform(tf::StampedTransform(map_to_published_frame, 
                                            t + ros::Duration(transform_offset), "map", published_frame_id));
}
 

tf::Transform PoseExtrapolator::getDeltaLaserPose(const tf::Transform& robot_on_odom)
{
    return corrected_laser_on_robot.inverse()*
           last_robot_on_odom.inverse()*
           robot_on_odom*
           corrected_laser_on_robot;
}

void PoseExtrapolator::addOdomMsg(const nav_msgs::OdometryConstPtr& msg)
{
    static bool first = true;

    newest_time = msg->header.stamp;

    if(first)
    {
        odom_temp_front = *msg;
        init_odom_front = true;
        first = false;
        return;
    }
    
    odom_queue_mutex.lock();
    odom_queue.push(*msg);

    while(odom_temp_front.header.stamp.toSec() < newest_time.toSec() - odom_queue_max_size && ros::ok())
    {
        odom_temp_front = odom_queue.front();
        odom_queue.pop();
    }
    odom_queue_mutex.unlock();
    // ROS_INFO_STREAM("Odom queue size : " <<odom_queue.size() + 1);
}

bool PoseExtrapolator::getRobotOnOdomFromQueue(const ros::Time& t,tf::Transform& transform)
{
    if(init_odom_front && odom_queue.size() > 0)
    {
        if(odom_temp_front.header.frame_id == odom_frame_id &&
           odom_temp_front.child_frame_id == robot_frame_id )
        {
            if(getOdom(transform,t))
                return true;
            else
                return false;
        }else{
            ROS_ERROR_STREAM("odom or robot frame id doesn't match to odom msg");
            return false;
        }
    }else{
        ROS_ERROR_STREAM("Not receive odom msg");
        return false;
    }
}

bool PoseExtrapolator::getOdom(tf::Transform& transform,const ros::Time& t)
{
    if(t < odom_temp_front.header.stamp)
    {
        ROS_ERROR_STREAM("Can not get past extrapolation from odom queue !!");
        return false;
    }    

    double wait_period = 0.0001; 
    int count = ceil(transform_tolerance/wait_period);

    bool can_do_interpolation = false;
    while(count > 0 && ros::ok())
    {
        if(t <= odom_queue.back().header.stamp)
        {
            can_do_interpolation = true;
            break;
        }
        ros::Duration(wait_period).sleep();
        count--;
    }

    if(!can_do_interpolation)
    {
        ROS_ERROR_STREAM(t << " " << odom_temp_front.header.stamp << " " << odom_queue.back().header.stamp);
        ROS_ERROR_STREAM("Can not get future extrapolation from odom queue !!");
        return false;
    }

    nav_msgs::Odometry odom_temp_back;
    odom_queue_mutex.lock();
    while(ros::ok() && odom_queue.size() > 0)
    {
        if(t <= odom_queue.front().header.stamp)
        {
            odom_temp_back = odom_queue.front();
            break;
        }
        odom_temp_front = odom_queue.front();
        odom_queue.pop();
    }
    odom_queue_mutex.unlock();

    tf::Vector3 start,end;
    tf::pointMsgToTF(odom_temp_front.pose.pose.position,start);
    tf::pointMsgToTF(odom_temp_back.pose.pose.position,end);
    
    tf::Quaternion start_q,end_q;
    tf::quaternionMsgToTF(odom_temp_front.pose.pose.orientation,start_q);
    tf::quaternionMsgToTF(odom_temp_back.pose.pose.orientation,end_q);

    double s = (t - odom_temp_front.header.stamp).toSec();
    double l = (odom_temp_back.header.stamp - odom_temp_front.header.stamp).toSec();

    tf::Vector3 inter = start.lerp(end,s/l);
    tf::Quaternion  inter_q = start_q.slerp(end_q,s/l);
    tf::Transform inter_tf;
    
    inter_tf.setOrigin(inter);
    inter_tf.setRotation(inter_q);

    transform = inter_tf;

    return true;
}

void PoseExtrapolator::odomCallback(const nav_msgs::OdometryConstPtr& msg)
{
    addOdomMsg(msg);
}

void PoseExtrapolator::openOdomSub()
{
    odom_sub = nh.subscribe(odom_topic,1000,&PoseExtrapolator::odomCallback,this);
}

bool PoseExtrapolator::getLastPose(tf::Pose& last_pose)
{
    if(pose_array.size() > 0)
    {
        last_pose = pose_array.back();
        return true;
    }else{
        return false;
    }
}


