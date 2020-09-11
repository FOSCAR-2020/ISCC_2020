#ifndef PURE_PURSUIT_PURE_PURSUIT_H
#define PURE_PURSUIT_PURE_PURSUIT_H

// ROS includes
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>

// C++ includes
#include <vector>
#include <obstacles.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <darknet_ros_msgs/BoundingBoxes.h>

namespace waypoint_follower
{
class PurePursuit
{
public:
  PurePursuit();
  ~PurePursuit();

  // for setting data
  void setLookaheadDistance(const double& ld)
  {
    lookahead_distance_ = ld;
  }
  void setWaypoints(const std::vector<std::pair<geometry_msgs::Point, int>>& wps);

  void setCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg)
  {
    current_pose_ = msg->pose;
  }

  // processing
  bool canGetCurvature(double* output_kappa);

  // for target point visualization
  geometry_msgs::Point getPoseOfNextTarget() const
  {
    return next_target_position_;
  }

  geometry_msgs::Point getCurrentPose() const
  {
    return current_position;
  }

// private:
  // variables
  int next_waypoint_number_;
  int current_idx;

  geometry_msgs::Point next_target_position_;
  geometry_msgs::Point current_position;

  double lookahead_distance_;
  geometry_msgs::Pose current_pose_;
  std::vector<std::pair<geometry_msgs::Point, int>> waypoints;
  int mode;
  int mission_flag;
  //bool current_idx_flag;

  // for main control
  int is_obstacle_detected;
  std::vector<Obstacle> obstacles;
  int static_obstacle_flag;
  bool straight_go_flag;
  bool left_go_flag;



  // functions
  double calcCurvature(geometry_msgs::Point target) const;
  void getNextWaypoint();


  bool reachMissionIdx(int target_idx);
};

// also from autoware
geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose);
double getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2);
tf::Vector3 point2vector(geometry_msgs::Point point);

}  // namespace waypoint_follower

#endif  // PURE_PURSUIT_PURE_PURSUIT_H
