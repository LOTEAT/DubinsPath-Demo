#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "dubins_planner/pose.hpp"
#include "dubins_planner/Pose2D.h"
#include "utils.cpp"

using dubins_planner::Pose2D;
using visualization_msgs::Marker;



void Pose::start_pose_callback(const PoseWithCovarianceStamped::ConstPtr& msg){
    Pose2D pose_msg;
    pose_copy(msg, pose_msg);
    this->start_pose_pub.publish(pose_msg);
    ROS_INFO_STREAM("Start pose, x: " << float_format(pose_msg.x) << " y: " << 
                float_format(pose_msg.y) << " theta:" << float_format(pose_msg.theta));
    Marker marker;
    create_marker(pose_msg, marker, 0, 1, 0);
    this->start_pose_vis_pub.publish(marker);
}

void Pose::goal_pose_callback(const PoseStamped::ConstPtr& msg){
    Pose2D pose_msg;
    pose_copy(msg, pose_msg);
    this->goal_pose_pub.publish(pose_msg);
    ROS_INFO_STREAM("Goal pose, x: " << float_format(pose_msg.x) << " y: " << 
                float_format(pose_msg.y) << " theta:" << float_format(pose_msg.theta));
    Marker marker;
    create_marker(pose_msg, marker, 1, 0, 0);
    this->goal_pose_vis_pub.publish(marker);
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose");
    Pose pose;
    ros::spin();
    return 0;
}
