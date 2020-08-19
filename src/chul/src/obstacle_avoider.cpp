#include <ros/ros.h>
#include "sensor_msgs/LaserScan.h"
#include <cstdlib>
#include <iostream>


#define MIN_SCAN_ANGLE_RAD  -70.0/180*M_PI
#define MAX_SCAN_ANGLE_RAD  +70.0/180*M_PI

using namespace std;

ros::Publisher control_pub;
ros::Subscriber obstacle_sub;

float dist;

bool isdetected = false;

void obstacleCB(const sensor_msgs::LaserScan::ConstPtr& laser) {

  int minIndex = 90;
	int maxIndex = 290;

    cout << laser->ranges.size() << endl;
    if(!isdetected) {
	    for(int currIndex = minIndex + 1; currIndex < maxIndex; currIndex++){

            if(laser->ranges[currIndex] < dist){
			    isdetected = true;
                ROS_INFO_STREAM("Stop Sign"    );
                break;
		    }
        }

	}
    else {
        int currIndex;

        for(currIndex = minIndex + 1; currIndex < maxIndex; currIndex++){

            if(laser->ranges[currIndex] < dist){

                break;
		    }
        }
        if(currIndex >= maxIndex) {
            isdetected = false;
            ROS_INFO_STREAM("Go Sign"   );
        }
    }




}

int main(int argc, char** argv) {
    ros::init(argc,argv,"dynamic_obstacle");
    ros::NodeHandle nh;

    dist = (float)atoi(argv[1]);

    obstacle_sub = nh.subscribe("/scan", 10, obstacleCB);

    ros::spin();

}
