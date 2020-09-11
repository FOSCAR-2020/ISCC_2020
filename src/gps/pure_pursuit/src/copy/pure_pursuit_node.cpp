// ROS Includes
#include <ros/ros.h>

// User defined includes
#include <pure_pursuit_core.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pure_pursuit");
  waypoint_follower::PurePursuitNode ppn;

  ppn.run(argv);

  return 0;
}
