#include <ros/ros.h>
#include <math.h>

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
