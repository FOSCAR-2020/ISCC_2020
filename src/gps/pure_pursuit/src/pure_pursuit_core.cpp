#include <vector>
#include <pure_pursuit_core.h>
#include <fstream>
#include <cstdlib>

#include <tf/transform_broadcaster.h>

namespace waypoint_follower
{
// Constructor
PurePursuitNode::PurePursuitNode()
  : private_nh_("~")
  , pp_()
  , LOOP_RATE_(30)
  , is_waypoint_set_(false)
  , is_pose_set_(false)
  , const_lookahead_distance_(4.0)
  , const_velocity_(3.0)
  , final_constant(1.5)
{
  initForROS();
}

// Destructor
PurePursuitNode::~PurePursuitNode() {}

void PurePursuitNode::initForROS()
{
  // ros parameter settings
  private_nh_.param("const_lookahead_distance", const_lookahead_distance_, 4.0);
  private_nh_.param("const_velocity", const_velocity_, 3.0);
  private_nh_.param("final_constant", final_constant, 1.5);
  nh_.param("vehicle_info/wheel_base", wheel_base_, 1.04);

  ROS_HOME = ros::package::getPath("pure_pursuit");

  // setup subscriber
  pose_sub = nh_.subscribe("current_pose", 1,
    &PurePursuitNode::callbackFromCurrentPose, this);

  // setup publisher
  drive_msg_pub = nh_.advertise<race::drive_values>("control_value", 1);
  steering_vis_pub = nh_.advertise<geometry_msgs::PoseStamped>("steering_vis", 1);

  // for visualization
  target_point_pub = nh_.advertise<geometry_msgs::PointStamped>("target_point", 1);
}

void PurePursuitNode::run(char** argv) {
  ROS_INFO_STREAM("pure pursuit start");

  // temp
  const_lookahead_distance_ = atof(argv[2]);
  const_velocity_ = atof(argv[3]);
  final_constant = atof(argv[4]);
  //////////////////////////

  ros::Rate loop_rate(LOOP_RATE_);
  while (ros::ok()) {
    ros::spinOnce();

    if (!is_waypoint_set_) {
      setPath(argv);
      pp_.setWaypoints(global_path);
    }

    if (!is_pose_set_) {
      loop_rate.sleep();
      continue;
    }

    pp_.setLookaheadDistance(computeLookaheadDistance());

    double kappa = 0;
    bool can_get_curvature = pp_.canGetCurvature(&kappa);

    publishDriveMsg(can_get_curvature, kappa);

    // target point visualization
    publishTargetPointVisualizationMsg();

    is_pose_set_ = false;
    loop_rate.sleep();
  }
}

void PurePursuitNode::publishDriveMsg(const bool& can_get_curvature, const double& kappa) const {
  race::drive_values drive_msg;
  drive_msg.throttle = can_get_curvature ? const_velocity_ : 0;

  double steering_radian = convertCurvatureToSteeringAngle(wheel_base_, kappa);
  drive_msg.steering = can_get_curvature ? (steering_radian * 180.0 / M_PI) * -1 * final_constant: 0;

  // std::cout << "steering : " << drive_msg.steering << "\tkappa : " << kappa <<std::endl;
  drive_msg_pub.publish(drive_msg);

  // for steering visualization
  publishSteeringVisualizationMsg(steering_radian);
}

double PurePursuitNode::computeLookaheadDistance() const {
  if (true) {
    return const_lookahead_distance_;
  }
}

void PurePursuitNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg) {
  pp_.setCurrentPose(msg);
  is_pose_set_ = true;
}

void PurePursuitNode::setPath(char** argv) {
  std::ifstream infile(ROS_HOME + "/paths/" + argv[1]);
  std::cout << ROS_HOME + "/paths/" + argv[1] << std::endl;
  geometry_msgs::Point p;

  double x, y;
  while(infile >> x >> y) {
    p.x = x;
    p.y = y;
    global_path.push_back(p);
  }

  is_waypoint_set_ = true;
}

void PurePursuitNode::publishTargetPointVisualizationMsg () {
  geometry_msgs::PointStamped target_point_msg;
  target_point_msg.header.frame_id = "/base_link";
  target_point_msg.header.stamp = ros::Time::now();
  target_point_msg.point = pp_.getPoseOfNextTarget();
  target_point_pub.publish(target_point_msg);
}

void PurePursuitNode::publishSteeringVisualizationMsg (const double& steering_radian) const {
  double yaw = atan2(2.0 * (pp_.current_pose_.orientation.w * pp_.current_pose_.orientation.z + pp_.current_pose_.orientation.x * pp_.current_pose_.orientation.y), 1.0 - 2.0 * (pp_.current_pose_.orientation.y * pp_.current_pose_.orientation.y + pp_.current_pose_.orientation.z * pp_.current_pose_.orientation.z));

  double steering_vis = yaw + steering_radian;
  geometry_msgs::Quaternion _quat = tf::createQuaternionMsgFromYaw(steering_vis);
  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "/base_link";
  pose.pose.position = pp_.current_pose_.position;
  pose.pose.orientation = _quat;
  steering_vis_pub.publish(pose);
}

double convertCurvatureToSteeringAngle(const double& wheel_base, const double& kappa) {
  return atan(wheel_base * kappa);
}

}  // namespace waypoint_follower
