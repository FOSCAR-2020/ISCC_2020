#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Bool.h>
#include <tf/transform_broadcaster.h>

#include <iostream>

static ros::Publisher pose_publisher;

static std_msgs::Bool gnss_stat_msg;

static geometry_msgs::PoseStamped _prev_pose;
static geometry_msgs::Quaternion _quat;
static double yaw;
// true if position history is long enough to compute orientation
static bool _orientation_ready = false;


static void GNSSCallback(geometry_msgs::Point data)
{
  static tf::TransformBroadcaster pose_broadcaster;
  tf::Transform pose_transform;
  tf::Quaternion pose_q;

  geometry_msgs::PoseStamped pose;
  pose.header.stamp = ros::Time::now();
  pose.header.frame_id = "/base_link";
  pose.pose.position.x = data.x;
  pose.pose.position.y = data.y;
  pose.pose.position.z = data.z;

  // set gnss_stat
  if (pose.pose.position.x == 0.0 || pose.pose.position.y == 0.0 || pose.pose.position.z == 0.0)
  {
    gnss_stat_msg.data = false;
  }
  else
  {
    gnss_stat_msg.data = true;
  }

  double distance = sqrt(pow(pose.pose.position.y - _prev_pose.pose.position.y, 2) +
                         pow(pose.pose.position.x - _prev_pose.pose.position.x, 2));
  //std::cout << "distance : " << distance << std::endl;

  if (distance > 0.2)
  {
    yaw = atan2(pose.pose.position.y - _prev_pose.pose.position.y, pose.pose.position.x - _prev_pose.pose.position.x);
    _quat = tf::createQuaternionMsgFromYaw(yaw);
    _prev_pose = pose;
    _orientation_ready = true;
  }

  if (_orientation_ready)
  {
    std::cout << "Yaw : " << yaw * 180 / M_PI << std::endl;
    pose.pose.orientation = _quat;
    pose_publisher.publish(pose);
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "coordinate2tfpose");
  ros::NodeHandle nh;
  
  pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("current_pose", 1);
  ros::Subscriber gnss_pose_subscriber = nh.subscribe("/utmk_coordinate", 1, GNSSCallback);

  ros::spin();
  return 0;
}
