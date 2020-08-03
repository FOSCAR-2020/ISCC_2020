#include <ros/ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <race/drive_values.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <string>
#include <stdlib.h>
#include <cstdlib>
#include <cstring>

int throttle;
int steering;

void ackermannMsgCallback(const ackermann_msgs::AckermannDriveStamped msg) {
	printf("%f\n", msg.drive.steering_angle);
	throttle = (int)(msg.drive.speed*60);
	steering = (int)(msg.drive.steering_angle*57.296);
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "ackermann_msg_to_drive_msg");
	ros::NodeHandle nh;

	ros::Subscriber ackermann_msg_sub = nh.subscribe("ackermann_cmd", 1, ackermannMsgCallback);
	ros::Publisher drive_msg_pub = nh.advertise<race::drive_values>("control_value", 1);

	while(ros::ok()) {
		race::drive_values drive_msg;
		drive_msg.throttle = throttle;
		drive_msg.steering = steering;
		drive_msg_pub.publish(drive_msg);
		ros::spinOnce();
	}
}
