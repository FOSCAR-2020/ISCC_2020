#ifndef PURE_PURSUIT_PURE_PURSUIT_H
#define PURE_PURSUIT_PURE_PURSUIT_H

// ROS includes
#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf/transform_datatypes.h>

// C++ includes
#include <vector>


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
  void setWaypoints(const std::vector<geometry_msgs::Point>& wps);

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

// private:
  // variables
  int next_waypoint_number_;
  geometry_msgs::Point next_target_position_;
  double lookahead_distance_;
  geometry_msgs::Pose current_pose_;
  std::vector<geometry_msgs::Point> waypoints;
  int mode;

  // for main control
  int is_obstacle_detected;

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
