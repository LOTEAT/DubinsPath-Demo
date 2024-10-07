#ifndef POSE_HPP_
#define POSE_HPP_

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include "dubins_planner/Pose2D.h"

using ros::Subscriber;
using ros::Publisher;
using ros::NodeHandle;
using geometry_msgs::PoseWithCovarianceStamped;
using geometry_msgs::PoseStamped;
using visualization_msgs::Marker;
using dubins_planner::Pose2D;

class Pose{
    private:
        Subscriber start_pose_sub, goal_pose_sub;
        Publisher start_pose_pub, goal_pose_pub;
        Publisher start_pose_vis_pub, goal_pose_vis_pub;
        NodeHandle nh;
    public:
        Pose() {
            start_pose_sub = nh.subscribe("/initialpose", 1, &Pose::start_pose_callback, this);
            goal_pose_sub = nh.subscribe("/move_base_simple/goal", 1, &Pose::goal_pose_callback, this);
            start_pose_pub = nh.advertise<Pose2D>("/pose/start_pose/pose", 1);
            goal_pose_pub = nh.advertise<Pose2D>("/pose/goal_pose/pose", 1);
            start_pose_vis_pub = nh.advertise<Marker>("/pose/start_pose/marker", 1);
            goal_pose_vis_pub = nh.advertise<Marker>("/pose/goal_pose/marker", 1);
        }

        void start_pose_callback(const PoseWithCovarianceStamped::ConstPtr& msg);
        void goal_pose_callback(const PoseStamped::ConstPtr& msg);
        
};

#endif // POSE_HPP_