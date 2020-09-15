#include <vector>
#include <pure_pursuit_core.h>
#include <fstream>
#include <cstdlib>
#include <unistd.h>
#include <algorithm>

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

// obstacle global variable
float tmp_yaw_rate = 0.0;

bool left_detected = false;
bool left_avoid = false;
bool right_detected = false;
bool right_avoid = false;

float target_dist = 3.0;
float min_dist = 1.0;

/* traffic Index manager */
const int tf_idx_1 = 1180;
const int tf_idx_2 = 1455;
const int tf_idx_3 = 2500;
const int tf_idx_4 = 2990;
const int tf_idx_5 = 3406;
const int tf_idx_6 = 5055;
const int tf_idx_7 = 5285;

/*************************/

// float tmp_distance = 100.0;
///////////////////////////////

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
  obstacle_sub2 = nh_.subscribe("detected_obs", 1,
    &PurePursuitNode::callbackFromObstacle2, this);
  traffic_light_sub = nh_.subscribe("darknet_ros/bounding_boxes",1,
    &PurePursuitNode::callbackFromTrafficLight, this);
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
    std::cout << "*******************************" << std::endl;

    // MODE 0 - START
    if(pp_.mode == 0){
      pp_.mission_flag = 0;
      const_lookahead_distance_ = 6;
      const_velocity_ = 10;
    }

    // MODE 1 - 주차
    // 주차 구간
    if (pp_.mode == 1) {
      if (pp_.mission_flag == 3) {
        const_lookahead_distance_ = 4;
        const_velocity_ = 6;
      }

      // first
      // int start_parking_idx = 260;
      // int end_parking_idx = 135;
      // int end_parking_backward_idx = 100;
      // int end_parking_full_steer_backward_idx = 60;

      // second
      int start_parking_idx = 280;
      int end_parking_idx = 87;
      int end_parking_backward_idx = 55;
      int end_parking_full_steer_backward_idx = 25;


      // third
      // int start_parking_idx = 300;
      // int end_parking_idx = 75;
      // int end_parking_backward_idx = 48;
      // int end_parking_full_steer_backward_idx = 15;

      // forth
      // int start_parking_idx = 320;
      // int end_parking_idx = 88;
      // int end_parking_backward_idx = 55;
      // int end_parking_full_steer_backward_idx = 20;

      // fifth
      // int start_parking_idx = 330;
      // int end_parking_idx = 80;
      // int end_parking_backward_idx = 50;
      // int end_parking_full_steer_backward_idx = 20;

      // sixth
      // int start_parking_idx = 345;
      // int end_parking_idx = 75;
      // int end_parking_backward_idx = 55;
      // int end_parking_full_steer_backward_idx = 25;


      // backward_speed : -5
      int backward_speed = -7;

      // //x < start_parking_idx < y
      if (pp_.mission_flag == 0 && pp_.next_waypoint_number_ >= start_parking_idx){
        pp_.setWaypoints(parking_path);
        const_lookahead_distance_ = 3;
        const_velocity_ = 3;
        pp_.mission_flag = 1;
      }
      // 주차 끝
      if(pp_.mission_flag == 1 && pp_.reachMissionIdx(end_parking_idx)){
        // 5초 멈춤
        for (int i = 0; i < 50; i++)
        {
          pulishControlMsg(0, 0);
          // 0.1초
          usleep(100000);
        }

        // 특정 지점까지는 그냥 후진
        while (!pp_.reachMissionIdx(end_parking_backward_idx)) {
          pulishControlMsg(backward_speed, 0);
          ros::spinOnce();
        }
        // 그 다음 지점까지는 풀조향 후진
        while (!pp_.reachMissionIdx(end_parking_full_steer_backward_idx)) {
          pulishControlMsg(backward_speed, 30);
          ros::spinOnce();
        }
        pp_.mission_flag = 2;
      }
      // 주차 빠져나오고 다시 global path로
      if (pp_.mission_flag == 2) {
        for (int i = 0; i < 30; i++) {
          pulishControlMsg(0, 0);
          // 0.1초
          usleep(100000);
        }

        pp_.setWaypoints(global_path);
        const_lookahead_distance_ = 4;
        const_velocity_ = 6;
        pp_.mission_flag = 3;
      }
    }


    // MODE 2 : 좌회전 구간, 신호등 X
    if (pp_.mode == 2) {
      pp_.mission_flag = 0;
      const_lookahead_distance_ = 3;
      const_velocity_ = 5;
    }

    // MODE 3 : 정적 장애물 구간(드럼통)
    if (pp_.mode == 3) {
      if(!pp_.static_obstacle_flag && left_detected)
      {
        ROS_INFO_STREAM("Detect First Obstacle & Change Flag to 1");
        pp_.static_obstacle_flag = 1;
        //pulishControlMsg(3, tmp_yaw_rate);
        pulishControlMsg(3, 20);
      }
      else if(pp_.static_obstacle_flag == 1 && left_detected && tmp_yaw_rate > 5 && tmp_yaw_rate < 45)
      {
        ROS_INFO_STREAM("Avoid Left Obstacle");
        //pulishControlMsg(3, tmp_yaw_rate);
        pulishControlMsg(3, 20);
      }
      else if(pp_.static_obstacle_flag == 1 && !left_detected && !left_avoid)
      {
        ROS_INFO_STREAM("Pass the First Obstacle & Change Flag to 2");
        left_avoid = true;
        pp_.static_obstacle_flag = 2;
        pulishControlMsg(3,-3);
      }
      else if(pp_.static_obstacle_flag == 2 && left_avoid && left_detected && tmp_yaw_rate >= 0 && tmp_yaw_rate < 45)
      {
        ROS_INFO_STREAM("Pass First Obstacle & Yaw to Straight");
        pulishControlMsg(3, -20);
      }
      else if(pp_.static_obstacle_flag == 2 && left_avoid && right_detected && tmp_yaw_rate < 0 && tmp_yaw_rate > -60)
      {
        ROS_INFO_STREAM("Change Flag to 3");
        pp_.static_obstacle_flag = 3;
        pulishControlMsg(3, -24);
      }
      else if(pp_.static_obstacle_flag == 3 && left_avoid && right_detected && tmp_yaw_rate < -5 && tmp_yaw_rate > -60)
      {
        ROS_INFO_STREAM("Avoid Right Obstacle");
        //publish(3, tmp_yaw_rate);
        pulishControlMsg(3, -24);
      }
      else if(pp_.static_obstacle_flag == 3 && left_avoid && !right_detected)
      {
        ROS_INFO_STREAM("Finish the Static Obstacle 1");
        pp_.static_obstacle_flag = 4;
        pulishControlMsg(3,0);
      }

      if(pp_.static_obstacle_flag > 0 && pp_.static_obstacle_flag < 4)
        continue;
      }

      // MODE 4
      if (pp_.mode == 4) {
        pp_.mission_flag = 0;
        const_lookahead_distance_ = 5;
        const_velocity_ = 8;
      }

      // MODE 5 : 신호등 정지
      if (pp_.mode == 5)
      {
        const_lookahead_distance_ = 5;
        const_velocity_ = 8;

        // 첫 신호등 인덱스 : tf_idx_1
        if(pp_.reachMissionIdx(tf_idx_1) && !pp_.straight_go_flag) {
          pulishControlMsg(0,0);
          continue;
        }
      }

      // MODE 6 : 교차로 직진
      if (pp_.mode == 6) {
        pp_.mission_flag = 0;
        const_lookahead_distance_ = 6;
        const_velocity_ = 10;
      }


      // MODE 7 : 좌회전 신호등
      if (pp_.mode == 7)
      {
        const_lookahead_distance_ = 5;
        const_velocity_ = 8;

        // 두번째 신호등 인덱스 : tf_idx_2
        if(pp_.reachMissionIdx(tf_idx_2) && !pp_.left_go_flag) {
          pulishControlMsg(0,0);
          continue;
        }
      }

      // MODE 8 : 교차로 좌회전
      if (pp_.mode == 8) {
        pp_.mission_flag = 0;
        const_lookahead_distance_ = 5;
        const_velocity_ = 8;
      }

      // MODE 9 : 동적 장애물 구간
      if (pp_.mode == 9) {
        const_lookahead_distance_ = 5;
        const_velocity_ = 8;

        if (pp_.is_obstacle_detected)
        {
          while(pp_.is_obstacle_detected) {

            pulishControlMsg(0, 0);

            std::cout << pp_.is_obstacle_detected << std::endl;
            // 1초
            //usleep(1000000);
            ros::spinOnce();
            loop_rate.sleep();
          }
          // pp_.mission_flag = 1;
        }
      }

      // MODE 10 : 교차로 우회전
      if (pp_.mode == 10 || pp_.mode == 11) {
        pp_.mission_flag = 0;
        const_lookahead_distance_ = 5;
        const_velocity_ = 8;
      }

    // MODE 12 : 정적 장애물 2 , Path switching  -- version 1
    if (pp_.mode == 12) {
      if (pp_.mission_flag == 0 && pp_.is_obstacle_detected) {
        pp_.setWaypoints(avoidance_path);
        const_lookahead_distance_ = 6;
        const_velocity_ = 4;
        pp_.mission_flag = 1;
      }
      // current_idx : 25
      else if (pp_.mission_flag == 1 && pp_.current_idx >= 300) {
        pp_.mission_flag = 2;
      }
      else if (pp_.mission_flag == 2 && pp_.is_obstacle_detected) {
        pp_.setWaypoints(global_path);
        const_lookahead_distance_ = 6;
        const_velocity_ = 4;
        pp_.mission_flag = 3;
      }

      // 세번째 신호등 인덱스 : tf_idx_3
      if(pp_.mission_flag == 3 && pp_.reachMissionIdx(tf_idx_3) && !pp_.straight_go_flag) {
        pulishControlMsg(0,0);
        continue;
      }
    }

    // MODE 12 : 정적 장애물 2 , Path switching  -- version 2
    // if (pp_.mode == 12) {
    //   if (pp_.mission_flag == 0 && pp_.reachMissionIdx(70)) {
    //     pp_.setWaypoints(avoidance_path);
    //     const_lookahead_distance_ = 3;
    //     const_velocity_ = 3;
    //     pp_.mission_flag = 1;
    //   }
    //
    //   if (pp_.mission_flag == 1 && pp_.is_obstacle_detected) {
    //     pp_.setWaypoints(global_path);
    //     const_lookahead_distance_ = 5;
    //     const_velocity_ = 3;
    //     pp_.mission_flag = 2;
    //   }
    // 신호등 인덱스 : 542
    // if(pp_.mission_flag == 3 && pp_.reachMissionIdx(542) && !pp_.straight_go_flag) {
    //   pulishControlMsg(0,0);
    //   continue;
    // }
    // }

    // MODE 13, 14 ,15
    if (pp_.mode == 13 ||pp_.mode == 14 || pp_.mode == 15) {
      pp_.mission_flag = 0;
      const_lookahead_distance_ = 6;
      const_velocity_ = 10;
    }

    // MODE 16 : 차선변경
    if (pp_.mode == 16) {
      pp_.mission_flag = 0;
      const_lookahead_distance_ = 5;
      const_velocity_ = 8;
    }

    // MODE 17 : 좌회전 신호등
    if (pp_.mode == 17)
    {
      const_lookahead_distance_ = 4;
      const_velocity_ = 6;

      // 네번째 신호등 인덱스 : tf_idx_4
      if(pp_.reachMissionIdx(tf_idx_4) && !pp_.left_go_flag) {
        pulishControlMsg(0,0);
        continue;
      }
    }

    // MODE 18 : 좌회전
    if (pp_.mode == 18) {
      pp_.mission_flag = 0;
      const_lookahead_distance_ = 5;
      const_velocity_ = 8;
    }

    // MODE 19 : 차선 변경 , 자회전 신호등
    if (pp_.mode == 19)
    {
      const_lookahead_distance_ = 4;
      const_velocity_ = 6;

      // 다섯번째 신호등 인덱스 : tf_idx_5
      if(pp_.reachMissionIdx(tf_idx_5) && !pp_.left_go_flag) {
        pulishControlMsg(0,0);
        continue;
      }
    }


    // MODE 20 : 직진 (부스터)
    if (pp_.mode == 20) {
      pp_.mission_flag = 0;
      const_lookahead_distance_ = 7;
      const_velocity_ = 12;
    }

    // MODE 21 : 신호등 없는 구간
    if (pp_.mode == 21) {
      pp_.mission_flag = 0;
      const_lookahead_distance_ = 6;
      const_velocity_ = 10;
    }

    // MODE 22 : 우회전
    if (pp_.mode == 22)
    {
      pp_.mission_flag = 0;
      const_lookahead_distance_ = 6;
      const_velocity_ = 8;
    }

    // MODE 23 : 신호등 구간
    if (pp_.mode == 23)
    {
      const_lookahead_distance_ = 6;
      const_velocity_ = 10;

      // 신호등 인덱스 : 1594
      // if(pp_.reachMissionIdx(1590) && !pp_.straight_go_flag) {
      //   pulishControlMsg(0,0);
      //   continue;
      // }
    }

    if(pp_.mode == 24)
    {
      const_lookahead_distance_ = 4;
      const_velocity_ = 6;
      // 다섯번째 신호등 인덱스 : tf_idx_6
      if(pp_.reachMissionIdx(tf_idx_6) && !pp_.straight_go_flag) {
        pulishControlMsg(0,0);
        continue;
      }
    }

    // MODE 24,25 : 신호등 직진 구간
    if (pp_.mode == 25)
    {
      const_lookahead_distance_ = 6;
      const_velocity_ = 10;
    }

    if(pp_.mode == 26)
    {
      const_lookahead_distance_ = 4;
      const_velocity_ = 6;

      // 일곱번신호등 인덱스 : tf_idx_7
      if(pp_.reachMissionIdx(tf_idx_7) && !pp_.straight_go_flag) {
        pulishControlMsg(0,0);
        continue;
      }
    }

    // MODE 26,27,28,29 : 직진 (부스터2)
    if (pp_.mode == 27 || pp_.mode == 28 || pp_.mode == 29) {
      pp_.mission_flag = 0;
      const_lookahead_distance_ = 7;
      const_velocity_ = 12;
      //final_constant = 1.5;
    }

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

void PurePursuitNode::callbackFromObstacle2(const avoid_obstacle::DetectedObstacles& msg) {
  // 전역 변수 tmp_yaw_rate
  tmp_yaw_rate = 0.0;

  left_detected = false;
  right_detected = false;
  pp_.obstacles.clear();

  // 모든 장애물 데이터 벡터에 저장
  for(unsigned int i = 0; i < msg.obstacles.size(); i++) {
      Obstacle obs = Obstacle(msg.obstacles[i].x, msg.obstacles[i].y, msg.obstacles[i].radius, msg.obstacles[i].true_radius);
      pp_.obstacles.push_back(obs);
  }

  // left avoid flag가 true 이면 인식 범위를 5로 늘림.
  if(left_avoid){
    target_dist = 5.0;
  }

  for(unsigned int i = 0; i < pp_.obstacles.size(); i++)
  {
      // 전방 0도 ~ 50도 확인
      if(pp_.obstacles[i].yaw_rate > 0 && pp_.obstacles[i].yaw_rate < 50.0)
      {
          // 전방 1m ~ target_distance 확인
          if (pp_.obstacles[i].dist < target_dist && pp_.obstacles[i].dist > min_dist){
              // ROS_INFO("Point [X,Y] : [%f, %f]", obstacles[i].x, obstacles[i].y);
              // ROS_INFO("Distance : [%f]     Yaw_Rate : [%f]", obstacles[i].dist, obstacles[i].yaw_rate);
              left_detected = true;
              tmp_yaw_rate = pp_.obstacles[i].yaw_rate;
              //tmp_distance = pp_.obstacles[i].dist;
          }
      }

      // 첫 장애물 통과후, 오른쪽 장애물 인식  => 전방 -60~0도 확인
      if(left_avoid && pp_.obstacles[i].yaw_rate > -60.0 && pp_.obstacles[i].yaw_rate <= 0)
      {
          if(pp_.obstacles[i].dist < target_dist)
          {
              // ROS_INFO("Point [X,Y] : [%f, %f]", obstacles[i].x, obstacles[i].y);
              // ROS_INFO("Distance : [%f]     Yaw_Rate : [%f]", obstacles[i].dist, obstacles[i].yaw_rate);
              right_detected = true;
              tmp_yaw_rate = pp_.obstacles[i].yaw_rate;
          }
      }

      // if(pp_.mission_flag == 4 && !right_detected) {
      //   right_avoid = true;
      // }
  }
}
void PurePursuitNode::callbackFromTrafficLight(const darknet_ros_msgs::BoundingBoxes& msg) {
  std::vector<darknet_ros_msgs::BoundingBox> traffic_lights = msg.bounding_boxes;
  std::sort(traffic_lights.begin(), traffic_lights.end(), compare);

  // for (unsigned int i = 0; i < traffic_lights.size(); i++) {
  //   int xmin = traffic_lights[i].xmin;
  //   int ymin = traffic_lights[i].ymin;
  //   int xmax = traffic_lights[i].xmax;
  //   int ymax = traffic_lights[i].ymax;
  //   int area;
  //
  //   std::cout << "index : " << i << std::endl;
  //   std::cout << "class name : "<<traffic_lights[i].Class << std::endl;
  //   // std::cout << "xmin : "<<traffic_lights[i].xmin << std::endl;
  //   // std::cout << "ymin : "<<traffic_lights[i].ymin << std::endl;
  //   // std::cout << "ymax : "<<traffic_lights[i].xmax << std::endl;
  //   // std::cout << "ymin : "<<traffic_lights[i].ymax << std::endl;
  //   area = (xmax - xmin) * (ymax - ymin);
  //   std::cout << "area : " << area << std::endl;
  // }


  if (traffic_lights[0].Class == "3 red" || traffic_lights[0].Class == "3 yellow" || traffic_lights[0].Class == "4 red" ||
      traffic_lights[0].Class == "4 yellow" || traffic_lights[0].Class == "4 red yellow")
  {
    pp_.straight_go_flag = false;
    pp_.left_go_flag = false;
  }
  else if (traffic_lights[0].Class == "3 green" || traffic_lights[0].Class == "4 green")
  {
    pp_.straight_go_flag = true;
    pp_.left_go_flag = false;
  }
  else if (traffic_lights[0].Class == "3 left" || traffic_lights[0].Class == "4 red left")
  {
    pp_.straight_go_flag = false;
    pp_.left_go_flag = true;
  }
  else if (traffic_lights[0].Class == "4 left go")
  {
    pp_.straight_go_flag = true;
    pp_.left_go_flag = true;
  }

  // traffic test
  // std::cout << "*******************" << std::endl << std::endl;
  // if (pp_.straight_go_flag){
  //   std::cout << "straight go" << std::endl;
  // }
  // if (pp_.left_go_flag) {
  //   std::cout << "left go" << std::endl;
  // }
}
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

bool compare(darknet_ros_msgs::BoundingBox a, darknet_ros_msgs::BoundingBox b) {
  int a_area = (a.ymax - a.ymin) * (a.xmax - a.xmin);
  int b_area = (b.ymax - b.ymin) * (b.xmax - b.xmin);

  return a_area > b_area ? true : false;
}

}  // namespace waypoint_follower
