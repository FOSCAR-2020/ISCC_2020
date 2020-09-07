#include <ros/ros.h>
#include <avoid_obstacle/DetectedObstacles.h>
#include <avoid_obstacle/TrueObstacles.h>
#include <iostream>
#include <vector>
#include <race/drive_values.h>
#include <math.h>

using namespace std;

ros::Publisher drive_msg_pub;

bool is_detected = false;

bool first_detected = false;
bool left_detected = false;
bool right_detected = false;
bool stop_flag = false;

float target_dist = 3.0;

int cnt = 0;
float tmp_yaw_rate = 0.0;

class My_Vector{
    public:
        float x;
        float y;
    
    My_Vector(float _x, float _y)
    {
        x = _x;
        y = _y;
    }

    void normalize()
    {
        float bottom = sqrt(x*x + y*y);
        x = x/bottom;
        y = y/bottom;
    }

};

class Obstacle{
    public:
        float x;
        float y;
        float radius;
        float true_radius;
        float dist;
        float yaw_rate;
    
    Obstacle(float _x, float _y, float _radius, float _true_radius)
    {
        x = _x;
        y = _y;
        radius = _radius;
        true_radius = _true_radius;
        dist = sqrt(x*x + y*y);

        My_Vector vec1 = My_Vector(_x, _y);
        
        vec1.normalize();

        float product_vec = vec1.x;
        float theta_rad = acos(product_vec);

        if (y < 0)
            yaw_rate = -theta_rad*180/M_PI;
        else
            yaw_rate = theta_rad*180/M_PI;
    }
};

void publishControlValue(int throttle, double steering) {
    race::drive_values drive_msg;
    drive_msg.throttle = throttle;
    drive_msg.steering = steering;
    drive_msg_pub.publish(drive_msg);

}

void detectedCallback(const avoid_obstacle::DetectedObstacles& msg){
    tmp_yaw_rate = 0.0;
    vector<Obstacle> obstacles;

    for(int i = 0; i < msg.obstacles.size(); i++)
    {   
        Obstacle obs = Obstacle(msg.obstacles[i].x, msg.obstacles[i].y, msg.obstacles[i].radius, msg.obstacles[i].true_radius);
        obstacles.push_back(obs);
    } 
    ROS_INFO("------------ Node Start ------------");
    ROS_INFO("1st Obs : [%d],  left detected : [%d]", first_detected, left_detected);
    ROS_INFO("right detected : [%d],  stop flag : [%d]", right_detected, stop_flag);

    for(int i = 0; i < obstacles.size(); i++)
    {   
        if(obstacles[i].yaw_rate < 45.0 && obstacles[i].yaw_rate > 0){
            if (obstacles[i].dist < target_dist){
                // ROS_INFO("Point [X,Y] : [%f, %f]", obstacles[i].x, obstacles[i].y);
                // ROS_INFO("Distance : [%f]     Yaw_Rate : [%f]", obstacles[i].dist, obstacles[i].yaw_rate);
                first_detected = true;
                tmp_yaw_rate = obstacles[i].yaw_rate;
            }
        }
        else if(left_detected && obstacles[i].yaw_rate > -45.0 && obstacles[i].yaw_rate <= 0)
        {
            if(obstacles[i].dist < target_dist)
            {
                // ROS_INFO("Point [X,Y] : [%f, %f]", obstacles[i].x, obstacles[i].y);
                // ROS_INFO("Distance : [%f]     Yaw_Rate : [%f]", obstacles[i].dist, obstacles[i].yaw_rate);
                right_detected = true;
                tmp_yaw_rate = obstacles[i].yaw_rate;
            }
        }
    }

    ROS_INFO("SIZE : [%d]", cnt);
}

void trueCallback(const avoid_obstacle::TrueObstacles& msg){
    is_detected = msg.detected;
    //ROS_INFO("Detected? : [%d]", is_detected);
    
    if (stop_flag)
    {
        publishControlValue(0, 0);
    }
    else if (left_detected && right_detected && tmp_yaw_rate < -5 && tmp_yaw_rate > -45)
    {
        ROS_INFO_STREAM("Avoid!! Left Turn!!");
        publishControlValue(3, -28);
    }
    else if (left_detected && tmp_yaw_rate > 5 && tmp_yaw_rate < 45){
        ROS_INFO_STREAM("Go Left");
        publishControlValue(3, -28);
    }
    else if (first_detected && tmp_yaw_rate > 5 && tmp_yaw_rate < 45){
        ROS_INFO_STREAM("Avoid!! Right Turn!!");
        publishControlValue(3, 28);
    }
    else{        
        if(first_detected)
        {
            target_dist = 5.0;
            left_detected = true;
        }
        // if(left_detected && right_detected)
        // {
        //     stop_flag = true;
        // }
        //target_dist = 3.0;
        publishControlValue(3, 0);
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_subscriber");
    ros::NodeHandle node;

    ros::Subscriber sub1 = node.subscribe("/detected_obs", 1000, detectedCallback);
    ros::Subscriber sub2 = node.subscribe("/true_obs", 1000, trueCallback);
    drive_msg_pub = node.advertise<race::drive_values>("control_value", 1);

    //ros::Publisher
    
    ros::spin();
}