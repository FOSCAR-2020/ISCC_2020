#include <ros/ros.h>
#include <serial/serial.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <race/drive_values.h>
#include <string>
#include <stdlib.h>
#include <cstdlib>
#include <cstring>
#include <sstream>


serial::Serial ser;

using namespace std;

unsigned char alive=0x00;
unsigned char gear = 0x00;
unsigned char speed_0 = 0x00;
unsigned char speed_1 = 0x00;
unsigned char steer_0 = 0x00;
unsigned char steer_1 = 0x00;
unsigned char front_brake = 0x01;
unsigned char estop = 0x00;

void serialCallback(const race::drive_values::ConstPtr& msg) {
	int steer_total = 0;
	unsigned int speed_total = 0;
	speed_total = abs(msg->throttle)*10;
	printf("msg->throttle : %d\n", msg->throttle);
	if(msg->throttle < 255 && msg->throttle > 0) { // forward
		gear = 0x00;
		speed_1 = speed_total;
		speed_0 = 0x00;
		front_brake = 0x00;
		estop = 0x00;
	} else if(msg->throttle > -255 && msg->throttle < 0) { // backward
		gear = 0x02;
		speed_1 = speed_total;
		speed_0 = 0x00;
		front_brake = 0x00;
		estop = 0x00;
	} else if(msg->throttle == 0) { // stop
		speed_0 = 0x00;
		speed_1 = 0x00;
		gear = 0x01;
		front_brake = 0x99;
		estop = 0x00;
	} else{
		gear = 0x01;
		speed_0 = 0x00;
		speed_1 = 0x00;
		front_brake = 0x33;
		estop = 0x01;
	}
	steer_total = (int)((msg->steering)*71.0);
	if(steer_total > 2000) steer_total = 2000;
	if(steer_total < -2000) steer_total = -2000;
	printf("steer : %d , speed : %u\n", steer_total, speed_total);
	steer_0 = steer_total >> 8;
	steer_1 = steer_total & 0xff;
}


int main (int argc, char** argv) {
	unsigned char str[14] = {0x53, 0x54, 0x58, 0x01, estop, 0x00, speed_0, speed_1, steer_0, steer_1, front_brake, alive, 0x0D, 0x0A};
	ros::init(argc, argv, "serial_node");
	ros::NodeHandle nh;

	ros::Subscriber control_value_sub = nh.subscribe("control_value", 10, serialCallback);
	ros::Publisher encoder_value_pub = nh.advertise<std_msgs::Int32>("encoder_value", 10);

	try {
		ser.setPort("/dev/ttyRS232");
		ser.setBaudrate(115200);
		serial::Timeout to = serial::Timeout::simpleTimeout(1000);
		ser.setTimeout(to);
		ser.open();
	} catch(serial::IOException& e) {
		ROS_ERROR_STREAM("Unable to open port");
		return -1;
	}

	if(ser.isOpen()) ROS_INFO_STREAM("Serial Port initialized");
	else return -1;

	ros::Rate loop_rate(50);
	size_t num_write = 14;
	size_t num_read = 18;


	// ser.readlines(3000);
	while(ros::ok()) {
		str[4]  = estop;
		str[5]  = gear;
		str[6]  = speed_0;
		str[7]  = speed_1;
		str[8]  = steer_0;
		str[9]  = steer_1;
		str[10] = front_brake;
		str[11] = alive;

		ser.write(str, num_write);

		std::string line;
		ser.readline(line, num_read);
		unsigned char encoder_values[4];
		encoder_values[0] = line[14];
		encoder_values[1] = line[13];
		encoder_values[2] = line[12];
		encoder_values[3] = line[11];
		int encoder_val = ((int)encoder_values[0] << 24) |
                              ((int)encoder_values[1] << 16) |
                              ((int)encoder_values[2] << 8 ) |
                              ((int)encoder_values[3] << 0);

		// cout << "encoder val : " << encoder_val << endl;

		std_msgs::Int32 encoder_msg;
		encoder_msg.data = encoder_val;

		encoder_value_pub.publish(encoder_msg);

		if(alive != 0xff) alive++;
		else alive = 0x00;

		loop_rate.sleep();
		ros::spinOnce();
	}
	ser.close();
}
