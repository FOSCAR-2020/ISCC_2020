#include <vector>
#include <pure_pursuit_core.h>
#include <fstream>

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
{
  initForROS();
}

// Destructor
PurePursuitNode::~PurePursuitNode() {}

// DONE
void PurePursuitNode::initForROS()
{
  // ros parameter settings
  private_nh_.param("const_lookahead_distance", const_lookahead_distance_, 4.0);
  private_nh_.param("const_velocity", const_velocity_, 3.0);
  nh_.param("vehicle_info/wheel_base", wheel_base_, 1.04);

  // setup subscriber
  pose_sub = nh_.subscribe("current_pose", 1,
    &PurePursuitNode::callbackFromCurrentPose, this);

  // setup publisher
  drive_msg_pub = nh_.advertise<race::drive_values>("control_value", 1);

  // for visualization
  target_point_pub = nh_.advertise<geometry_msgs::PointStamped>("target_point", 1);
}
///////////////////////////////////

// DONE
void PurePursuitNode::run()
{
  ROS_INFO_STREAM("pure pursuit start");
  ros::Rate loop_rate(LOOP_RATE_);
  while (ros::ok())
  {
    ros::spinOnce();

    if (!is_waypoint_set_)
    {
      setPath();
      pp_.setWaypoints(global_path);
      int len = global_path.size();
    }    
    
    if (!is_pose_set_)
    {
      loop_rate.sleep();
      continue;
    }

    pp_.setLookaheadDistance(computeLookaheadDistance());

    double kappa = 0;
    bool can_get_curvature = pp_.canGetCurvature(&kappa);

    // for target point visualization
    geometry_msgs::PointStamped target_point_msg;
    target_point_msg.header.frame_id = "/base_link";
    target_point_msg.header.stamp = ros::Time::now();
    target_point_msg.point = pp_.getPoseOfNextTarget();
    target_point_pub.publish(target_point_msg);
    ///////////////////////////////////

    publishDriveMsg(can_get_curvature, kappa);

    is_pose_set_ = false;

    loop_rate.sleep();
  }
}
///////////////////////////////////

// DONE
void PurePursuitNode::publishDriveMsg(
  const bool& can_get_curvature, const double& kappa) const
{
  race::drive_values drive_msg;
  drive_msg.throttle = can_get_curvature ? const_velocity_ : 0;
  
  drive_msg.steering =
    can_get_curvature ? convertCurvatureToSteeringAngle(wheel_base_, kappa) * 180.0 / M_PI * 71.0 * -1 : 0;

  std::cout << "steering : " << drive_msg.steering / 71.0 << "\tkappa : " << kappa <<std::endl;
  drive_msg_pub.publish(drive_msg);
}
///////////////////////////////////

// DONE
double PurePursuitNode::computeLookaheadDistance() const
{
  if (true)
  {
    return const_lookahead_distance_;
  }
}
///////////////////////////////////

// DONE
void PurePursuitNode::callbackFromCurrentPose(
  const geometry_msgs::PoseStampedConstPtr& msg)
{
  pp_.setCurrentPose(msg);
  is_pose_set_ = true;
}
///////////////////////////////////

// maybe DONE
void PurePursuitNode::setPath() {
  std::ifstream infile("/home/foscar/ISCC_2019/src/pure_pursuit_test/pure_pursuit/src/path.txt");
  geometry_msgs::Point p;

  double x, y;
  while(infile >> x >> y) {
    p.x = x;
    p.y = y;
    global_path.push_back(p);
  }
  
  is_waypoint_set_ = true;
}
///////////////////////////////////

// DONE
double convertCurvatureToSteeringAngle(
  const double& wheel_base, const double& kappa)
{
  return atan(wheel_base * kappa);
}
///////////////////////////////////

}  // namespace waypoint_follower
