#include <ros/ros.h>
#include <cmath>
#include <Eigen/Dense>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include "dubins_planner/dubins_planner.hpp"
#include "dubins_planner/Pose2D.h"

#define PI_2  2*M_PI

inline float mod2pi(double angel) {return fmod(angel + PI_2, PI_2);}

void DubinsPlanner::coord_transform(){
    Vector3d start_coord_homo(this->start_pose.x, this->start_pose.y, 1);
    Vector3d goal_coord_homo(this->goal_pose.x, this->goal_pose.y, 1);
    Matrix3d translation_mat = Matrix3d::Identity();
    translation_mat(0, 2) = -start_coord_homo(0);
    translation_mat(1, 2) = -start_coord_homo(1);
    double delta_x = goal_coord_homo(0) - start_coord_homo(0);
    double delta_y = goal_coord_homo(1) - start_coord_homo(1);
    double yaw = atan2(delta_y, delta_x);
    double rot_angle = yaw;
    Matrix3d rotation_mat;
    rotation_mat << cos(rot_angle), sin(rot_angle), 0,
                    -sin(rot_angle), cos(rot_angle), 0, 
                    0, 0, 1;
    this->transformation_mat = rotation_mat * translation_mat;
    Matrix3d rotation_mat_inv = this->transformation_mat.inverse();
    this->transformation_mat_inv = rotation_mat_inv;
    this->transform_start_pose = this->transformation_mat * start_coord_homo;
    this->transform_start_pose(2) = mod2pi(this->start_pose.theta - rot_angle);
    this->transform_goal_pose = this->transformation_mat * goal_coord_homo;
    this->transform_goal_pose(2) = mod2pi(this->goal_pose.theta - rot_angle);
}

vector<Vector3d> DubinsPlanner::sample_points(Vector3d start_pose, double step, double length, string mode){
    vector<Vector3d> points;
    double walk_length = 0;
    double start_x = start_pose(0), start_y = start_pose(1), yaw = start_pose(2);
    if(mode == "left"){
        double center_x = -sin(yaw) * this->rho + start_x;
        double center_y = cos(yaw) * this->rho + start_y;
        while(walk_length < length){
            Vector3d point;
            point(0) = sin(yaw + walk_length) * this->rho + center_x;
            point(1) = -cos(yaw + walk_length) * this->rho + center_y;
            point(2) = 1;
            walk_length += step;
            points.push_back(point);
        }
    } else if (mode == "straight") {
        while(walk_length < length){
            Vector3d point;
            point(0) = cos(yaw) * walk_length * this->rho + start_x;
            point(1) = sin(yaw) * walk_length * this->rho + start_y;
            point(2) = 1;
            walk_length += step;
            points.push_back(point);
        }
    } else if (mode == "right") {
        double center_x = sin(yaw) * this->rho + start_x;
        double center_y = -cos(yaw) * this->rho + start_y;
        while(walk_length < length){
            Vector3d point;
            point(0) = -sin(yaw - walk_length) * this->rho + center_x;
            point(1) = cos(yaw - walk_length) * this->rho + center_y;
            point(2) = 1;
            walk_length += step;
            points.push_back(point);
        }
    } else {
        ROS_ERROR_STREAM("There is no mode \'" << mode << "\' in Dubins model." );
    }
    return points;
}




void DubinsPlanner::plan_path(){
    if(this->is_start_pose_ready && this->is_goal_pose_ready){
        this->coord_transform();
        ROS_INFO("Coordinate transformation...");
        ROS_INFO_STREAM("After coordinate transformation, start pose, x: " << float_format(this->transform_start_pose(0)) << " y: " << 
            float_format(this->transform_start_pose(1)) << " theta: " << float_format(this->transform_start_pose(2)));
        ROS_INFO_STREAM("After coordinate transformation, goal pose, x: " << float_format(this->transform_goal_pose(0)) << " y: " << 
            float_format(this->transform_goal_pose(1)) << " theta: " << float_format(this->transform_goal_pose(2)));
        PathCost lsl_cost = this->lsl_path();
        PathCost rsr_cost = this->rsr_path();
        PathCost rsl_cost = this->rsl_path();
        PathCost lsr_cost = this->lsr_path();
        vector<Vector3d> lsl_path = this->generate_path(lsl_cost);
        vector<Vector3d> rsr_path = this->generate_path(rsr_cost);
        vector<Vector3d> rsl_path = this->generate_path(rsl_cost);
        vector<Vector3d> lsr_path = this->generate_path(lsr_cost);
        lsl_cost.scale(this->rho);
        rsr_cost.scale(this->rho);
        rsl_cost.scale(this->rho);
        lsr_cost.scale(this->rho);
        print_cost(lsl_cost);
        print_cost(rsr_cost);
        print_cost(rsl_cost);
        print_cost(lsr_cost);
        double total_cost = lsl_cost.total + rsr_cost.total + rsl_cost.total + lsr_cost.total;
        double norm_factor = total_cost / lsl_cost.total + total_cost / rsr_cost.total + total_cost / rsl_cost.total + total_cost / lsr_cost.total;
        Marker lsl_marker_msg, rsr_marker_msg, rsl_marker_msg, lsr_marker_msg;
        create_line(lsl_path, lsl_marker_msg, this->path_header, 1.0, 1.0, 0, total_cost / lsl_cost.total / norm_factor);
        create_line(rsr_path, rsr_marker_msg, this->path_header, 1.0, 0, 1.0, total_cost / rsr_cost.total / norm_factor);
        create_line(rsl_path, rsl_marker_msg, this->path_header, 0, 1.0, 1.0, total_cost / rsl_cost.total / norm_factor);
        create_line(lsr_path, lsr_marker_msg, this->path_header, 1.0, 1.0, 1.0, total_cost / lsr_cost.total / norm_factor);
        this->lsl_path_pub.publish(lsl_marker_msg);
        this->rsr_path_pub.publish(rsr_marker_msg);
        this->rsl_path_pub.publish(rsl_marker_msg);
        this->lsr_path_pub.publish(lsr_marker_msg);
    }
}

PathCost DubinsPlanner::lsl_path(){
    double alpha = this->transform_start_pose(2);
    double d = this->transform_goal_pose(0) / this->rho;
    double beta = this->transform_goal_pose(2);
    double tmp = atan2((cos(beta) - cos(alpha)), (d + sin(alpha) - sin(beta)));
    double t_lsl = fmod(fmod(-alpha + tmp, PI_2) + PI_2, PI_2);
    double p_lsl = sqrt(2 + pow(d, 2) - 2 * cos(alpha - beta) + 2 * d * (sin(alpha) - sin(beta)));
    double q_lsl = fmod(fmod(beta - tmp, PI_2) + PI_2, PI_2);
    double path = t_lsl + p_lsl + q_lsl;
    return PathCost(path, t_lsl, "left", p_lsl, "straight", q_lsl, "left");
}

PathCost DubinsPlanner::rsr_path(){
    double alpha = this->transform_start_pose(2);
    double d = this->transform_goal_pose(0) / this->rho;
    double beta = this->transform_goal_pose(2);
    double tmp = atan2((cos(alpha) - cos(beta)), (d - sin(alpha) + sin(beta)));
    double t_rsr = mod2pi(alpha - tmp);
    double p_rsr = sqrt(2 + pow(d, 2) - 2 * cos(alpha - beta) + 2 * d * (sin(beta) - sin(alpha)));
    double q_rsr = mod2pi(-beta + tmp);
    double path = t_rsr + p_rsr + q_rsr;
    return PathCost(path, t_rsr, "right", p_rsr, "straight", q_rsr, "right");
}

PathCost DubinsPlanner::rsl_path(){
    double alpha = this->transform_start_pose(2);
    double d = this->transform_goal_pose(0) / this->rho;
    double beta = this->transform_goal_pose(2);
    double p_rsl = sqrt(-2 + pow(d, 2) + 2 * cos(alpha - beta) - 2 * d * (sin(beta) + sin(alpha)));
    double tmp = -atan2((cos(alpha) + cos(beta)), (d - sin(alpha) - sin(beta))) + atan2(2.0, p_rsl);
    double t_rsl = mod2pi(alpha + tmp);
    double q_rsl = mod2pi(beta + tmp);
    double path = t_rsl + p_rsl + q_rsl;
    return PathCost(path, t_rsl, "right", p_rsl, "straight", q_rsl, "left");
}

PathCost DubinsPlanner::lsr_path(){
    double alpha = this->transform_start_pose(2);
    double d = this->transform_goal_pose(0) / this->rho;
    double beta = this->transform_goal_pose(2);
    double p_lsr = sqrt(-2 + pow(d, 2) + 2 * cos(alpha - beta) + 2 * d * (sin(beta) + sin(alpha)));
    double tmp = atan2((-cos(alpha) - cos(beta)), (d + sin(alpha) + sin(beta))) - atan2(-2.0, p_lsr);
    double t_lsr = mod2pi(-alpha + tmp);
    double q_lsr = mod2pi(-mod2pi(beta) + tmp);
    double path = t_lsr + p_lsr + q_lsr;
    return PathCost(path, t_lsr, "left", p_lsr, "straight", q_lsr, "right");
}


Vector3d DubinsPlanner::get_end_pose(Vector3d point, double yaw, double path_length, string mode){
    double new_yaw;
    if(mode == "left"){
        new_yaw = yaw + path_length / 1.0;
    } else if (mode == "straight"){
        new_yaw = yaw;
    } else if (mode == "right") {
        new_yaw = yaw - path_length / 1.0;
    }
    Vector3d end_pose(point(0), point(1), new_yaw);
    return end_pose;
}

vector<Vector3d> DubinsPlanner::generate_path(PathCost path_cost){
    vector<Vector3d> path;
    vector<Vector3d> path1 = this->sample_points(this->transform_start_pose, 0.01, path_cost.path1, path_cost.mode1);
    path.insert(path.end(), path1.begin(), path1.end());
    Vector3d path1_end_pose = this->get_end_pose(path1.back(), this->transform_start_pose(2), path_cost.path1, path_cost.mode1);
    vector<Vector3d> path2 = this->sample_points(path1_end_pose, 0.01, path_cost.path2, path_cost.mode2);
    path.insert(path.end(), path2.begin(), path2.end());
    Vector3d path2_end_pose = this->get_end_pose(path2.back(), path1_end_pose(2), path_cost.path2, path_cost.mode2);
    vector<Vector3d> path3 = this->sample_points(path2_end_pose, 0.01, path_cost.path3, path_cost.mode3);
    path.insert(path.end(), path3.begin(), path3.end());
    for (vector<Vector3d>::iterator it = path.begin(); it != path.end(); ++it) {
        *it = this->transformation_mat_inv * (*it);
    }
    return path;
}


int main(int argc, char** argv)
{
    ros::init(argc, argv, "planner");
    double rho;
    ros::param::get("/robot/rho", rho);
    DubinsPlanner dubins_planner(rho);
    ros::spin();
    return 0;
}
