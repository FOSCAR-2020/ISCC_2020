#include <pure_pursuit.h>

namespace waypoint_follower
{
// Constructor
PurePursuit::PurePursuit() : next_waypoint_number_(-1), current_idx(-1), lookahead_distance_(0), mode(0), mission_flag(0), is_obstacle_detected(false), static_obstacle_flag(0),
                             straight_go_flag(false), left_go_flag(false) {}

// Destructor
PurePursuit::~PurePursuit() {}

void PurePursuit::setWaypoints(const std::vector<std::pair<geometry_msgs::Point, int>>& wps)
{
  waypoints = wps;
  next_waypoint_number_ = -1;
}

double PurePursuit::calcCurvature(geometry_msgs::Point target) const
{
  double kappa = 0;
  double denominator = pow(getPlaneDistance(target, current_pose_.position), 2);
  double numerator = 2 * calcRelativeCoordinate(target, current_pose_).y;

  if (denominator != 0)
  {
    kappa = numerator / denominator;
  }

  return kappa;
}

void PurePursuit::getNextWaypoint()
{
  int path_size = static_cast<int>(waypoints.size());
  bool current_idx_flag = false;
  // if waypoints are not given, do nothing.
  // std::cout << path_size << std::endl;
  if (path_size == 0)
  {
    next_waypoint_number_ = -1;
    return;
  }
  if (next_waypoint_number_ == -1) {
    float min_distance = 9999999;
    for (int i = 0; i < path_size; i++) {
      float current_distance = getPlaneDistance(waypoints.at(i).first, current_pose_.position);
      if (min_distance > current_distance) {
        min_distance = current_distance;
        next_waypoint_number_ = i;
      }
    }
    current_idx_flag = true;
    // std::cout <<"********FIRST WAYPOINT*********" << std::endl;
    // std::cout << waypoints.at(next_waypoint_number_).x << " " << waypoints.at(next_waypoint_number_).y << std::endl;
    // std::cout <<"*******************************" << std::endl;
  }

  // 만약 처음 초기화를 한 상황이라면
  if (current_idx_flag) {
    current_idx = next_waypoint_number_;
  }

  // look for current vehicle in dex
  for (int i = current_idx; i < path_size; i++) {
    // if search waypoint is the last
    if (i == (path_size - 1)) {
      //ROS_INFO("search waypoint is the last");
      current_idx = i;
      break;
    }

    if (getPlaneDistance(waypoints.at(i).first, current_pose_.position) > 4) {
      int path_size2 = static_cast<int>(waypoints.size());
      float min_distance2 = 9999999;
      for (int j = 0; j < path_size2; j++) {
        float current_distance2 = getPlaneDistance(waypoints.at(j).first, current_pose_.position);
        if (min_distance2 > current_distance2) {
          min_distance2 = current_distance2;
          current_idx = j;
        }
      }
      break;
    }
    if (getPlaneDistance(waypoints.at(i).first, current_pose_.position) > 1) {
      current_idx = i;
      mode = waypoints.at(i).second;
      break;
    }
  }

  // look for the next waypoint.
  for (int i = next_waypoint_number_; i < path_size; i++)
    // if search waypoint is the last
    {
    if (i == (path_size - 1))
    {
      ROS_INFO("search waypoint is the last");
      next_waypoint_number_ = i;
      return;
    }

    // if there exists an effective waypoint
    if (getPlaneDistance(waypoints.at(i).first, current_pose_.position) > lookahead_distance_)
    {
      next_waypoint_number_ = i;
      return;
    }
  }

  // if this program reaches here , it means we lost the waypoint!
  next_waypoint_number_ = -1;
  return;
}

bool PurePursuit::canGetCurvature(double* output_kappa)
{
  // search next waypoint
  getNextWaypoint();

  if (next_waypoint_number_ == -1)
  {
    ROS_INFO("lost next waypoint");
    return false;
  }

  next_target_position_ = waypoints.at(next_waypoint_number_).first;
  current_position = waypoints.at(current_idx).first;

  // std::cout << std::fixed;
  // std::cout.precision(5);
  // std::cout << "target_index :" <<next_waypoint_number_ << std::endl;
  // std::cout << "target_coordinate : " << next_target_position_.x << " " << next_target_position_.y << std::endl;

  *output_kappa = calcCurvature(next_target_position_);
  return true;
}

bool PurePursuit::reachMissionIdx(int misson_idx) {
  geometry_msgs::Point mission_position = waypoints.at(misson_idx).first;
  double distance = getPlaneDistance(mission_position, current_pose_.position);
  if (distance < 0.8)
   return true;
  else
   return false;
}


// calculation relative coordinate of point from current_pose frame
geometry_msgs::Point calcRelativeCoordinate(geometry_msgs::Point point_msg, geometry_msgs::Pose current_pose)
{
  tf::Transform inverse;
  tf::poseMsgToTF(current_pose, inverse);
  tf::Transform transform = inverse.inverse();

  tf::Point p;
  pointMsgToTF(point_msg, p);
  tf::Point tf_p = transform * p;
  geometry_msgs::Point tf_point_msg;
  pointTFToMsg(tf_p, tf_point_msg);

  return tf_point_msg;
}

// distance between target 1 and target2 in 2-D
double getPlaneDistance(geometry_msgs::Point target1, geometry_msgs::Point target2)
{
  tf::Vector3 v1 = point2vector(target1);
  v1.setZ(0);
  tf::Vector3 v2 = point2vector(target2);
  v2.setZ(0);
  return tf::tfDistance(v1, v2);
}

tf::Vector3 point2vector(geometry_msgs::Point point)
{
  tf::Vector3 vector(point.x, point.y, point.z);
  return vector;
}

}  // namespace waypoint_follower
