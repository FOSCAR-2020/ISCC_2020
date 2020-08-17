#ifndef PURE_PURSUIT_PURE_PURSUIT_CORE_H
#define PURE_PURSUIT_PURE_PURSUIT_CORE_H

// ROS includes
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <ros/package.h>

// User defined includes
#include <race/drive_values.h>
#include <pure_pursuit.h>

#include <vector>
#include <memory>
#include <string>

namespace waypoint_follower
{

class PurePursuitNode
{
public:
  PurePursuitNode();
  ~PurePursuitNode();

  void run(char** argv);

private:
  // handle
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;

  // class
  PurePursuit pp_;

  // publisher
  ros::Publisher drive_msg_pub;
  ros::Publisher steering_vis_pub;

  ros::Publisher target_point_pub;

  // subscriber
  ros::Subscriber pose_sub;

  // constant
  const int LOOP_RATE_;  // processing frequency

  // variables
  bool is_waypoint_set_, is_pose_set_, is_velocity_set_;
  double wheel_base_;
  double const_lookahead_distance_;  // meter
  double const_velocity_;            // km/h

  std::vector<geometry_msgs::Point> global_path;
  std::string ROS_HOME;

  // callbacks
  void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
  //void callbackFromCurrentVelocity(
  //  const geometry_msgs::TwistStampedConstPtr& msg);

  // initializer
  void initForROS();

  // functions
  void publishDriveMsg(
    const bool& can_get_curvature, const double& kappa) const;

  double computeLookaheadDistance() const;
  
  // set wayPath
  void setPath(char** argv);
};

double convertCurvatureToSteeringAngle(
  const double& wheel_base, const double& kappa);

}  // namespace waypoint_follower

#endif  // PURE_PURSUIT_PURE_PURSUIT_CORE_H
