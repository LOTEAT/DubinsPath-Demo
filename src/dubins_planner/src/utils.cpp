#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_listener.h>
#include <ros/ros.h>
#include <iomanip>
#include <sstream>
#include <Eigen/Dense>
#include <visualization_msgs/Marker.h>
#include "dubins_planner/Pose2D.h"


using Eigen::Vector3d;
using dubins_planner::Pose2D;
using geometry_msgs::PoseWithCovarianceStamped;
using geometry_msgs::PoseStamped;
using geometry_msgs::Point;
using geometry_msgs::Quaternion;
using std_msgs::Header;
using visualization_msgs::Marker;
using std::vector;
using std::string;
using std::fixed;
using std::setprecision;
using std::ostringstream;

struct PathCost {
    double total;
    double path1;
    string mode1;
    double path2;
    string mode2;
    double path3;
    string mode3;
    PathCost(){}
    PathCost(double total, double path1, string mode1, double path2, string mode2, double path3, string mode3): 
    total(total), path1(path1), mode1(mode1), path2(path2), mode2(mode2), path3(path3), mode3(mode3) {}
    void scale(double factor){
        total *= factor;
        path1 *= factor;
        path2 *= factor;
        path3 *= factor;
    }
    bool operator<(const PathCost& other) const {
        if (this->total < other.total)
            return true;
        return false;
    }
    bool operator>(const PathCost& other) const {
        if (this->total > other.total)
            return true;
        return false;
    }
    bool operator==(const PathCost& other) const {
        if (this->total == other.total)
            return true;
        return false;
    }
};


void pose_copy(const PoseWithCovarianceStamped::ConstPtr& src, Pose2D& tgt){
    Header header = src->header;
    Point position = src->pose.pose.position;
    Quaternion orientation = src->pose.pose.orientation;
    Pose2D pose2d_msg;
    tgt.header = header;
    tgt.x = position.x;
    tgt.y = position.y;
    tgt.theta = tf::getYaw(orientation);
    tgt.orientation = orientation;
    return;
}

void pose_copy(const PoseStamped::ConstPtr& src, Pose2D& tgt){
    Header header = src->header;
    Point position = src->pose.position;
    Quaternion orientation = src->pose.orientation;
    Pose2D pose2d_msg;
    tgt.header = header;
    tgt.x = position.x;
    tgt.y = position.y;
    tgt.theta = tf::getYaw(orientation);
    tgt.orientation = orientation;
    return;
}

void create_marker(const Pose2D& pose_msg, Marker& marker_msg, double r=0, double g=1, double b=0){
    marker_msg.header = pose_msg.header;
    marker_msg.type = Marker::ARROW;
    marker_msg.action = Marker::ADD;
    marker_msg.pose.position.x = pose_msg.x;
    marker_msg.pose.position.y = pose_msg.y;
    marker_msg.pose.position.z = 0.5;
    marker_msg.pose.orientation = pose_msg.orientation;
    marker_msg.lifetime = ros::Duration();
    marker_msg.scale.x = 1;
    marker_msg.scale.y = 0.05;
    marker_msg.scale.z = 0.05;
    marker_msg.color.a = 1.0;
    marker_msg.color.r = r;
    marker_msg.color.g = g;
    marker_msg.color.b = b;
}

string float_format(double value, int precision = 3){
    ostringstream oss;
    oss << fixed << setprecision(precision) << value;
    return oss.str();
}


void create_line(const vector<Vector3d>& path, Marker& marker_msg, const Header& path_header, double r=0, double g=1, double b=0, double alpha=1.0){
    marker_msg.header = path_header;
    marker_msg.type = Marker::LINE_STRIP;
    marker_msg.action = Marker::ADD;

    marker_msg.lifetime = ros::Duration();
    marker_msg.scale.x = 0.05;
    marker_msg.scale.y = 0.05;
    marker_msg.scale.z = 0.05;
    marker_msg.color.a = alpha;
    marker_msg.color.r = r;
    marker_msg.color.g = g;
    marker_msg.color.b = b;
    for (vector<Vector3d>::const_iterator it = path.begin(); it != path.end(); ++it) {
        Point p;
        p.x = (*it)(0);
        p.y = (*it)(1);
        p.z = 0.5;
        marker_msg.points.push_back(p);
    }
}

void print_cost(PathCost path_cost){
    ROS_INFO_STREAM(
        "\n--------------------------------\n" <<
        "Total Path Cost: " << path_cost.total << "\n" <<
        "First Stage: " << path_cost.mode1 << " Cost: " << path_cost.path1 << "\n" <<
        "Second Stage: " << path_cost.mode2 << " Cost: " << path_cost.path2 << "\n" <<
        "Third Stage: " << path_cost.mode3 << " Cost: " << path_cost.path3 << "\n" <<
        "--------------------------------"
    );
}
