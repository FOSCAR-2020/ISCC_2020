#include <ros/ros.h>

#include <race/lane_info.h>
#include <nav_msgs/Odometry.h>
#include <race/mode.h>
#include <race/drive_values.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int32.h>

#include <vector>
#include <fstream>
#include <cstring>
#include <iostream>
#include <cmath>
#include <cstdlib>
#include <mutex>

#define _USE_MATH_DEFINES

struct Point {
    float x;
    float y;
    Point() {x = 0; y = 0;}
    Point(float _x, float _y) : x(_x), y(_y) {}
};

enum { BASE, };


float path_arrived_threshold = 1.5;
float yaw_refresh_threshold = 0.2;

int mode = BASE;
int current_path_index = 0;
std::vector<Point> path;
bool is_path_set = false;
Point current_position;
Point prev_position;
bool is_lane_detected = false;
// float yaw = 0.0;
ros::Publisher drive_msg_pub;

float cal_distance(const Point A, const Point B) {
    return sqrt((A.x - B.x)*(A.x - B.x) + (A.y - B.y)*(A.y - B.y));
}

float getAngle(std::vector<Point> v1, std::vector<Point> v2) {
    float x1, y1, x2, y2;
    x1 = v1[1].x - v1[0].x;
    y1 = v1[1].y - v1[0].y;
    x2 = v2[1].x - v2[0].x;
    y2 = v2[1].y - v2[0].y;

    // float u1 = sqrt(x1*x1 + y1*y1);
    // float u2 = sqrt(x2*x2 + y2*y2);
    // x1 /= u1;
    // y1 /= u1;
    // x2 /= u2;
    // y2 /= u2;

    // std::cout << x1 << ' ' << y1 << ' ' << x2 << ' ' << y2 << std::endl;
    return asin((x1*y2-y1*x2)/((cal_distance(v1[0], v1[1])*(cal_distance(v2[0], v2[1]))))) * 180.0 / M_PI;

}

void set_path() {
    std::string HOME = std::getenv("HOME") ? std::getenv("HOME") : ".";
    std::ifstream infile(HOME+"/ISCC_2019/src/race/src/path5.txt");
    std::string line;

    float min_dis = 9999999;
    float x, y;
    while(infile >> x >> y) {
        path.push_back(Point(x, y));
        std::cout.precision(11);
	std::cout << std::fixed << path.back().x << ' ' << path.back().y << std::endl;
	float cur_dis = cal_distance(path.back(), current_position);
        if(min_dis > cur_dis) {
	    min_dis = cur_dis;
            current_path_index = path.size()-1;
        }
    }
    ROS_INFO("path initialized, index : %d, position : %f %f", current_path_index, current_position.x, current_position.y);
    
    is_path_set = true;
}


void odom_callback(const nav_msgs::Odometry::ConstPtr& odom) {
    current_position.x = odom->pose.pose.position.x;
    current_position.y = odom->pose.pose.position.y;

    if(!is_path_set) {
        set_path();
    }

    if(mode == BASE && is_path_set) {
        float steering, throttle=6;
        std::vector<Point> v1, v2;
        v1.push_back(prev_position);
        // v1.push_back(Point(current_position.x+cos(yaw), current_position.y+sin(yaw)));
        v1.push_back(current_position);
        v2.push_back(current_position);
        v2.push_back(path[current_path_index]);
        steering = getAngle(v1, v2);
        
        race::drive_values drive_msg;
        drive_msg.throttle = (int)throttle;
        drive_msg.steering = (int)steering;
        if(steering >= 28) steering = 28;
	if(steering <= -28) steering = -28;

        // ROS_INFO("steering : %f", steering);
        std::cout << "steering : " << steering << std::endl;

        drive_msg_pub.publish(drive_msg);
    }
    std::cout << current_path_index << ' ' << cal_distance(current_position, prev_position) << std::endl;
    if(cal_distance(current_position, path[current_path_index]) < path_arrived_threshold) current_path_index++;

    if(cal_distance(current_position, prev_position) > yaw_refresh_threshold) {
        prev_position.x = current_position.x;
        prev_position.y = current_position.y;
    }
}

void imu_callback(const std_msgs::Float64::ConstPtr& msg) {
    // yaw = msg->data;
}

void lane_info_callback(const race::lane_info::ConstPtr& msg) {

}

void mode_callback(const race::mode::ConstPtr& msg) {
    mode = msg->mode;
}


int main(int argc, char** argv) {
    ros::init(argc, argv, "central_controller_node");
    ros::NodeHandle nh;

    ros::Subscriber odom_sub = nh.subscribe("odom", 1, odom_callback);
    ros::Subscriber imu_sub = nh.subscribe("imu/yaw", 1, imu_callback);
    ros::Subscriber lane_info_sub = nh.subscribe("lane_info", 1, lane_info_callback);
    ros::Subscriber mode_sub = nh.subscribe("mode", 1, mode_callback);
    drive_msg_pub = nh.advertise<race::drive_values>("control_value", 1);

    // To Do 초기 Odometry 설정


    ros::spin();
}
