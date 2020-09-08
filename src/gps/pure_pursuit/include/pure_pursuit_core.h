#ifndef PURE_PURSUIT_PURE_PURSUIT_CORE_H
#define PURE_PURSUIT_PURE_PURSUIT_CORE_H

// ROS includes
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <ros/package.h>

// for main control
// #include <>

// User defined includes
#include <race/drive_values.h>
#include <avoid_obstacle/DetectedObstacles.h>
#include <avoid_obstacle/TrueObstacles.h>
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
  ros::Publisher current_point_pub;

  // subscriber
  ros::Subscriber pose_sub;
  ros::Subscriber obstacle_sub;
  ros::Subscriber obstacle_sub2;
  ros::Subscriber traffic_light_sub;
  // ros::Subscriber lane_sub;

  // constant
  const int LOOP_RATE_;  // processing frequency

  // variables
  bool is_waypoint_set_, is_pose_set_, is_velocity_set_;
  double wheel_base_;
  double const_lookahead_distance_;  // meter
  double const_velocity_;            // km/h
  double final_constant;

  std::vector<std::pair<geometry_msgs::Point, int>> global_path;
  std::vector<std::pair<geometry_msgs::Point, int>> parking_path;
  std::vector<std::pair<geometry_msgs::Point, int>> avoidance_path;

  std::string ROS_HOME;

  // callbacks
  void callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg);
  //void callbackFromCurrentVelocity(
  //  const geometry_msgs::TwistStampedConstPtr& msg);

  // for main control
  void callbackFromObstacle(const avoid_obstacle::TrueObstacles& msg);
  void callbackFromObstacle2(const avoid_obstacle::DetectedObstacles& msg);
  // void callbackFromTrafficLight(const {msg_type}& msg);
  // void callbackFromLane(const {msg_type}& msg);


  // initializer
  void initForROS();

  // functions
  void publishPurePursuitDriveMsg(const bool& can_get_curvature, const double& kappa) const;
  void pulishControlMsg(double throttle, double steering) const;

  void publishTargetPointVisualizationMsg ();
  void publishCurrentPointVisualizationMsg ();
  void publishSteeringVisualizationMsg (const double& steering_radian) const;

  double computeLookaheadDistance() const;

  // set wayPath
  void setPath(char** argv);
};

double convertCurvatureToSteeringAngle(
  const double& wheel_base, const double& kappa);

void path_split(const std::string& str, std::vector<std::string>& cont,
		const std::string& delim);

}  // namespace waypoint_follower

#endif  // PURE_PURSUIT_PURE_PURSUIT_CORE_H
