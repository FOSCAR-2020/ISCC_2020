#include <ros/ros.h>
#include <avoid_obstacle/DetectedObstacles.h>
#include <avoid_obstacle/TrueObstacles.h>
#include <iostream>
#include <vector>
#include <race/drive_values.h>

using namespace std;

ros::Publisher drive_msg_pub;
bool is_detected = false;

class Obstacle{
    public:
        float x;
        float y;
        float radius;
        float true_radius;
    
    Obstacle(float _x, float _y, float _radius, float _true_radius)
    {
        x = _x;
        y = _y;
        radius = _radius;
        true_radius = _true_radius;
    }
};

void publishControlValue(int throttle, double steering) {
    race::drive_values drive_msg;
    drive_msg.throttle = throttle;
    drive_msg.steering = steering;
    drive_msg_pub.publish(drive_msg);

}

void detectedCallback(const avoid_obstacle::DetectedObstacles& msg){
    
    vector<Obstacle> obstacles;
    int cnt = 0;
    for(int i = 0; i < msg.obstacles.size(); i++)
    {   
        cnt++;
        Obstacle obs = Obstacle(msg.obstacles[i].x, msg.obstacles[i].y, msg.obstacles[i].radius, msg.obstacles[i].true_radius);
        obstacles.push_back(obs);
    } 
    ROS_INFO("SIZE : [%d]", cnt);

    for(int i = 0; i < obstacles.size(); i++)
    {   
        ROS_INFO("Point [X,Y] : [%f, %f]", obstacles[i].x, obstacles[i].y);
    }
}

void trueCallback(const avoid_obstacle::TrueObstacles& msg){
    is_detected = msg.detected;
    ROS_INFO("Detected? : [%d]", is_detected);

    if (is_detected)
        publishControlValue(0, 0);
    else
        publishControlValue(3, 0);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "obstacle_subscriber");
    ros::NodeHandle node;

    ros::Subscriber sub1 = node.subscribe("/detected_obs", 1000, detectedCallback);
    ros::Subscriber sub2 = node.subscribe("/true_obs", 1000, trueCallback);
    drive_msg_pub = node.advertise<race::drive_values>("control_value", 1);
    
    ros::spin();
}