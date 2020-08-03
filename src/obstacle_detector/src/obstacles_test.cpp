#include <ros/ros.h>

#include <obstacle_detector/Obstacles.h>
#include <geometry_msgs/Point.h>
#include <iostream>
#include <cmath>

float cal_distance(float A, float B) {
	return sqrt(A*A+B*B);
}

void obstacle_callback(const obstacle_detector::Obstacles::ConstPtr& obstacles) {
	geometry_msgs::Point car_point;
	geometry_msgs::Point obstacle_point;
	geometry_msgs::Point final_point;
	geometry_msgs::Point gps_car;

	car_point.x = 0.0, car_point.y = 0.0;
	gps_car.x = 30.0, gps_car.y = 30.0;
	
	float min_distance = 9999999;
	int idx = -1;


	for(int i = 0 ; i < obstacles->circles.size() ; i++) {
		float cur_distance = cal_distance(obstacles->circles[i].center.x, obstacles->circles[i].center.y);
		if(min_distance > cur_distance) {
			min_distance = cur_distance;
			idx = i;
			obstacle_point = obstacles->circles[i].center;
		}
	}
 	
	float diag = sqrt(pow(obstacle_point.x, 2) + pow(obstacle_point.y, 2));
	float dist = 100; //temp dist
	float diag2 = sqrt(pow(diag, 2) + pow(dist, 2));

	float theta1 = atan2(obstacle_point.y, obstacle_point.x);
	float theta2 = atan2(diag, dist);
	float yaw = 50; //temp yaw
	float angle = 90.0 - yaw;

	final_point.x = sqrt(pow(diag2, 2) / (1 + pow(tan(theta1 + theta2), 2)));
	final_point.y = final_point.x * tan(theta1 + theta2);


	float mat[2][2] = {
		{cos(angle), -sin(angle)},
		{sin(angle), cos(angle)}
	};
	float xy[2][1] = {
		{final_point.x},
		{final_point.y}
	};
	
	// xy = mat*xy;
	
	//if(obstacles->circles.size() > 0)
	//	std::cout << xy.x + gps_car.x << ' ' << xy.y + gps_car.y << std::endl;
	//else {
	//	std::cout << "not found" << std::endl;
	//}
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "test_node");

	ros::NodeHandle nh;

	ros::Subscriber obstacles_sub = nh.subscribe("obstacles", 10, obstacle_callback);

	ros::spin();
}
