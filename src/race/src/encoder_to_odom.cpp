#include <ros/ros.h>
#include <iostream>
#include <race/enc_values.h>
#include <cmath>
#include <boost/shared_ptr.hpp>
#include <tf/transform_broadcaster.h>

using namespace std;


struct loc {
	float xpos;
	float ypos;
	float yaw;
	loc() {}
	loc(float _xpos, float _ypos, float _yaw) : xpos(_xpos), ypos(_ypos), yaw(_yaw) {}
	loc& operator = (const loc &l) {
		xpos = l.xpos;
		ypos = l.ypos;
		yaw = l.yaw;
	}
};

bool state = false;
loc pre_loc;
loc cur_loc;
int pre_enc_val;
int cur_enc_val;
float yaw_ = 0;
boost::shared_ptr<tf::TransformBroadcaster> tf_pub_;
ros::Publisher odom_pub_;

void encoderCallback(const race::enc_values::ConstPtr& msg) {
	cur_enc_val = msg->enc_val;
	int cur_steering_angle = msg->steering;

	if(!state) {
		state = true;
		pre_enc_val = cur_enc_val;
	}

	ros::Duration dt = ;

	yaw_ += (cur_enc_val - pre_enc_val)/dt;
	float r = (cur_enc_val - pre_enc_val)/yaw;
	float y_ = r - r*cos(yaw);
	float x_ = r*sin(yaw);

	pre_enc_val = cur_enc_val;


	nav_msgs::Odometry::Ptr odom(new nav_msgs::Odometry);
	odom->pose.pose.position.x = x_;
	odom->pose.pose.position.y = y_;
	odom->pose.pose.orientation.x = 0.0;
	odom->pose.pose.orientation.y = 0.0;
	odom->pose.pose.orientation.z = sin(yaw_/2.0);
	odom->pose.pose.orientation.w = cos(yaw_/2.0);

	odom->pose.covariance[0]  = 0.2; 
	odom->pose.covariance[7]  = 0.2; 
	odom->pose.covariance[35] = 0.4; 

	odom->twist.twist.linear.x = (cur_enc_val - pre_enc_val)/dt;
	odom->twist.twist.linear.y = 0.0;
	odom->twist.twist.angular.z = (cur_enc_val - pre_enc_val)/dt;

	
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "encoder_to_odom_node");
	ros::NodeHandle nh;

	ros::Subscriber encoder_value_sub = nh.subscribe("encoder_value", 10, encoderCallback);
	ros::spin();
}