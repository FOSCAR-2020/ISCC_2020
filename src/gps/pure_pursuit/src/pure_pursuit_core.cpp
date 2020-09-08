#include <vector>
#include <pure_pursuit_core.h>
#include <fstream>
#include <cstdlib>
#include <unistd.h>

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

  // for main control
  obstacle_sub = nh_.subscribe("true_obs", 1,
    &PurePursuitNode::callbackFromObstacle, this);
  // obstacle_sub = nh_.subscribe("{traffic_light_topic_name}",1,
  //   &PurePursuitNode::callbackFromTrafficLight, this);
  // obstacle_sub = nh_.subscribe("{lane_topic_name}", 1,
  //   &PurePursuitNode::callbackFromLane, this);


  // setup publisher
  drive_msg_pub = nh_.advertise<race::drive_values>("control_value", 1);
  steering_vis_pub = nh_.advertise<geometry_msgs::PoseStamped>("steering_vis", 1);

  // for visualization
  target_point_pub = nh_.advertise<geometry_msgs::PointStamped>("target_point", 1);
  current_point_pub = nh_.advertise<geometry_msgs::PointStamped>("current_point", 1);
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

    // target point visualization
    publishTargetPointVisualizationMsg();
    publishCurrentPointVisualizationMsg();

    std::cout << "*******************************" << std::endl;
    std::cout << "current index : " << pp_.current_idx << std::endl;
    std::cout << "current mode : " << pp_.mode << std::endl;
    std::cout << "target index : " << pp_.next_waypoint_number_ << std::endl;

    // 주차 구간
    // if (pp_.mode == 1) {
    //   int start_parking_idx = 110;
    //   int end_parking_idx = 109;
    //   int end_parking_backward_idx = 85;
    //   int end_parking_full_steer_backward_idx = 50;
    //   int backward_speed = -5;
    //
    //   if (pp_.mission_flag == 0 && pp_.next_waypoint_number_ >= start_parking_idx){
    //     pp_.setWaypoints(parking_path);
    //     const_lookahead_distance_ = 3;
    //     const_velocity_ = 3;
    //     pp_.mission_flag = 1;
    //   }
    //   // 주차 끝
    //   if (pp_.mission_flag == 1 && pp_.reachMissionIdx(end_parking_idx)){
    //     // 5초 멈춤
    //     for (int i = 0; i < 50; i++) {
    //       pulishControlMsg(0, 0);
    //       // 0.1초
    //       usleep(100000);
    //     }
    //
    //     // 특정 지점까지는 그냥 후진
    //     while (!pp_.reachMissionIdx(end_parking_backward_idx)) {
    //       pulishControlMsg(backward_speed, 0);
    //       ros::spinOnce();
    //     }
    //     // 그 다음 지점까지는 풀조향 후진
    //     while (!pp_.reachMissionIdx(end_parking_full_steer_backward_idx)) {
    //       pulishControlMsg(backward_speed, 30);
    //       ros::spinOnce();
    //     }
    //     pp_.mission_flag = 2;
    //   }
    //   // 주차 빠져나오고 다시 global path로
    //   if (pp_.mission_flag == 2) {
    //     for (int i = 0; i < 30; i++) {
    //       pulishControlMsg(0, 0);
    //       // 0.1초
    //       usleep(100000);
    //     }
    //
    //     pp_.setWaypoints(global_path);
    //     const_lookahead_distance_ = 4;
    //     const_velocity_ = 6;
    //     pp_.mission_flag = 3;
    //   }
    // }
    // /////////////////////////////////////////////
    //
    // // 자회전 구간
    // if (pp_.mode == 2) {
    //   pp_.mission_flag = 0;
    //   const_lookahead_distance_ = 3;
    //   const_velocity_ = 3;
    // }
    //
    // // 정적 장애물 구간
    // if (pp_.mode == 3) {
    //   if (pp_.mission_flag == 0 && pp_.is_obstacle_detected) {
    //     pp_.setWaypoints(avoidance_path);
    //     const_lookahead_distance_ = 3;
    //     const_velocity_ = 3;
    //     pp_.mission_flag = 1;
    //   }
    //   if (pp_.mission_flag == 1 && pp_.reachMissionIdx(40)) {
    //     pp_.mission_flag = 2;
    //   }
    //
    //   if (pp_.mission_flag == 2 && pp_.is_obstacle_detected) {
    //     pp_.setWaypoints(global_path);
    //     const_lookahead_distance_ = 6;
    //     const_velocity_ = 3;
    //     pp_.mission_flag = 3;
    //     pp_.mode = 4;
    //   }
    // }
    // /////////////////////////////////////////////
    //
    // // 동적 장애물 구간
    // if (pp_.mode == 4) {
    //   if (pp_.is_obstacle_detected) {
    //     while(pp_.is_obstacle_detected) {
    //
    //       pulishControlMsg(0, 0);
    //
    //       std::cout << pp_.is_obstacle_detected << std::endl;
    //       // 1초
    //
    //       //usleep(1000000);
    //       ros::spinOnce();
    //       loop_rate.sleep();
    //     }
    //   }
    // }
    /////////////////////////////////////////////

    publishPurePursuitDriveMsg(can_get_curvature, kappa);

    is_pose_set_ = false;
    loop_rate.sleep();
  }
}

void PurePursuitNode::publishPurePursuitDriveMsg(const bool& can_get_curvature, const double& kappa) const {
  double throttle_ = can_get_curvature ? const_velocity_ : 0;

  double steering_radian = convertCurvatureToSteeringAngle(wheel_base_, kappa);
  double steering_ = can_get_curvature ? (steering_radian * 180.0 / M_PI) * -1 * final_constant: 0;

  // std::cout << "steering : " << steering_ << "\tkappa : " << kappa <<std::endl;
  pulishControlMsg(throttle_, steering_);

  // for steering visualization
  publishSteeringVisualizationMsg(steering_radian);
}

double PurePursuitNode::computeLookaheadDistance() const {
  if (true) {
    return const_lookahead_distance_;
  }
}

void PurePursuitNode::pulishControlMsg(double throttle, double steering) const
{
  race::drive_values drive_msg;
  drive_msg.throttle = throttle;
  drive_msg.steering = steering;
  drive_msg_pub.publish(drive_msg);
}


void PurePursuitNode::callbackFromCurrentPose(const geometry_msgs::PoseStampedConstPtr& msg) {
  pp_.setCurrentPose(msg);
  is_pose_set_ = true;
}

void PurePursuitNode::setPath(char** argv) {
  std::vector<std::string> paths;
  path_split(argv[1], paths, ",");
  std::ifstream global_path_file(ROS_HOME + "/paths/" + paths[0] + ".txt");
  //std::cout << ROS_HOME + "/paths/" + argv[1] << std::endl;

  // path.txt
  // <x, y, mode>
  geometry_msgs::Point p;
  double x, y;
  int mode;

  while(global_path_file >> x >> y >> mode) {
    p.x = x;
    p.y = y;
    //pp_.mode = mode;

    global_path.push_back(std::make_pair(p, mode));
    //std::cout << "global_path : " << global_path.back().x << ", " << global_path.back().y << std::endl;
  }
  if (paths.size() == 3) {
    std::ifstream parking_path_file(ROS_HOME + "/paths/" + paths[1] + ".txt");
    while(parking_path_file >> x >> y >> mode) {
      p.x = x;
      p.y = y;
      parking_path.push_back(std::make_pair(p, mode));
      //std::cout << "parking_path : " << parking_path.back().x << ", " << parking_path.back().y << std::endl;
    }

    std::ifstream avoidance_path_file(ROS_HOME + "/paths/" + paths[2] + ".txt");
    while(avoidance_path_file >> x >> y >> mode) {
      p.x = x;
      p.y = y;
      avoidance_path.push_back(std::make_pair(p, mode));
      // std::cout << "avoidance_path : " << avoidance_path.back().x << ", " << parking_path.back().y << std::endl;
    }
  }
  else if (paths.size() == 2) {
    std::ifstream parking_path_file(ROS_HOME + "/paths/" + paths[1] + ".txt");
    while(parking_path_file >> x >> y >> mode) {
      p.x = x;
      p.y = y;
      parking_path.push_back(std::make_pair(p, mode));
      // std::cout << "parking_path : " << parking_path.back().x << ", " << parking_path.back().y << std::endl;
    }
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

void PurePursuitNode::publishCurrentPointVisualizationMsg () {
  geometry_msgs::PointStamped current_point_msg;
  current_point_msg.header.frame_id = "/base_link";
  current_point_msg.header.stamp = ros::Time::now();
  current_point_msg.point = pp_.getCurrentPose();
  current_point_pub.publish(current_point_msg);
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

// for main control
void PurePursuitNode::callbackFromObstacle(const avoid_obstacle::TrueObstacles& msg) {
  pp_.is_obstacle_detected = msg.detected;
  //std::cout << "msg.detected : " << msg.detected << std::endl;
}
// void callbackFromTrafficLight(const {msg_type}& msg)
// void callbackFromLane(const {msg_type}& msg)

double convertCurvatureToSteeringAngle(const double& wheel_base, const double& kappa) {
  return atan(wheel_base * kappa);
}

void path_split(const std::string& str, std::vector<std::string>& cont,
		const std::string& delim)
{
    size_t prev = 0, pos = 0;
    do
    {
        pos = str.find(delim, prev);
        if (pos == std::string::npos) pos = str.length();
        std::string token = str.substr(prev, pos-prev);
        if (!token.empty()) cont.push_back(token);
        prev = pos + delim.length();
    }
    while (pos < str.length() && prev < str.length());
}

}  // namespace waypoint_follower
