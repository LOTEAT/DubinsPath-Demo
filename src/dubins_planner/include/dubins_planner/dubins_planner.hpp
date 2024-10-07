#ifndef DUBINS_PLANNER_HPP_
#define DUBINS_PLANNER_HPP_

#include <Eigen/Dense>
#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include "dubins_planner/Pose2D.h"
#include "../../src/utils.cpp"

using std::vector;
using std::string;
using ros::Subscriber;
using ros::Publisher;
using ros::NodeHandle;
using Eigen::Vector3d;
using Eigen::Matrix3d;
using visualization_msgs::Marker;
using geometry_msgs::Point;
using dubins_planner::Pose2D;
using std_msgs::Header;


class DubinsPlanner
{
    private:
        // turning radius
        double rho;
        // a variable to save header msg
        Header path_header;
        Subscriber start_pose_sub, goal_pose_sub;
        Publisher lsl_path_pub, rsr_path_pub, rsl_path_pub, lsr_path_pub;
        NodeHandle nh;
        Pose2D start_pose, goal_pose;
        bool is_start_pose_ready;
        bool is_goal_pose_ready;
        Vector3d transform_start_pose;
        Vector3d transform_goal_pose;
        Matrix3d transformation_mat;
        Matrix3d transformation_mat_inv;
        vector<Vector3d> sample_points(Vector3d start_pose, double step, double path_length, string mode);
        Vector3d get_end_pose(Vector3d point, double yaw, double path_length, string mode);
        vector<Vector3d> generate_path(PathCost path_cost);
        void coord_transform();        
        PathCost lsl_path();
        PathCost rsr_path();
        PathCost rsl_path();
        PathCost lsr_path();
        
    public:
        DubinsPlanner(double rho = 1.0) {
            this->rho = rho;
            this->is_start_pose_ready = false;
            this->is_goal_pose_ready = false;
            this->start_pose_sub = nh.subscribe("/pose/start_pose/pose", 1, &DubinsPlanner::start_pose_callback, this);
            this->goal_pose_sub = nh.subscribe("/pose/goal_pose/pose", 1, &DubinsPlanner::goal_pose_callback, this);
            this->lsl_path_pub = nh.advertise<Marker>("path/lsl/marker", 1);
            this->rsr_path_pub = nh.advertise<Marker>("path/rsr/marker", 1);
            this->rsl_path_pub = nh.advertise<Marker>("path/rsl/marker", 1);
            this->lsr_path_pub = nh.advertise<Marker>("path/lsr/marker", 1);
        }

        void start_pose_callback(const Pose2D::ConstPtr& msg){
            this->start_pose = *msg;
            this->is_start_pose_ready = true;
            this->path_header = msg->header;
            this->plan_path();
        }
        void goal_pose_callback(const Pose2D::ConstPtr& msg){
            this->goal_pose = *msg;
            this->is_goal_pose_ready = true;
            this->path_header = msg->header;
            this->plan_path();
        }
        void plan_path();
};

#endif // DUBINS_PLANNER_HPP_
