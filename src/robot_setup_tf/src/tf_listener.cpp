#include <ros/ros.h>
#include <geometry_msgs/PointStamped.h>
#include <tf/transform_listener.h>

void transformPoint(const tf::TransformListener& listener){
  //we'll create a point in the base_laser frame that we'd like to transform to the base_link frame
  geometry_msgs::PointStamped lidar_point;
  lidar_point.header.frame_id = "base_lidar";

  geometry_msgs::PointStamped imu_point;
  imu_point.header.frame_id = "base_imu";

  geometry_msgs::PointStamped gps_point;
  gps_point.header.frame_id = "base_gps";

  //we'll just use the most recent transform available for our simple example
  lidar_point.header.stamp = ros::Time();

  imu_point.header.stamp = ros::Time();

  gps_point.header.stamp = ros::Time();

  //just an arbitrary point in space
  lidar_point.point.x = 1.0;
  lidar_point.point.y = 0.2;
  lidar_point.point.z = 0.0;

  imu_point.point.x = 0.0;
  imu_point.point.y = 0.0;
  imu_point.point.z = 0.0;

  gps_point.point.x = 0.0;
  gps_point.point.y = 0.0;
  gps_point.point.z = 0.0;


  try{
    geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", lidar_point, base_point);

    ROS_INFO("base_lidar: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        lidar_point.point.x, lidar_point.point.y, lidar_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  
      //geometry_msgs::PointStamped base_point;
    listener.transformPoint("base_link", imu_point, base_point);

    ROS_INFO("base_imu: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        imu_point.point.x, imu_point.point.y, imu_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());

    listener.transformPoint("base_link", imu_point, base_point);

    ROS_INFO("base_gps: (%.2f, %.2f. %.2f) -----> base_link: (%.2f, %.2f, %.2f) at time %.2f",
        gps_point.point.x, gps_point.point.y, gps_point.point.z,
        base_point.point.x, base_point.point.y, base_point.point.z, base_point.header.stamp.toSec());
  
  }
  catch(tf::TransformException& ex){
    ROS_ERROR("Received an exception trying to transform a point from \"base_seonsor\" to \"base_link\": %s", ex.what());
  }
}

int main(int argc, char** argv){
  ros::init(argc, argv, "robot_tf_listener");
  ros::NodeHandle n;

  tf::TransformListener listener(ros::Duration(30));

  //we'll transform a point once every second
  ros::Timer timer = n.createTimer(ros::Duration(1.0), boost::bind(&transformPoint, boost::ref(listener)));

  ros::spin();

}
