#include <iostream>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <pso.h>

using namespace std;
using namespace cv;

float PSO::getScore(std::vector<float> values)
{

    return 0.0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_node");
    ROS_INFO("Started transform node");
    ros::NodeHandle n1, n2;
    ros::Rate rate(20);
//    ros::Subscriber laserScanSubscriber = n1.subscribe("/calibration_plane", 1, laserscanCB);
//    ros::Subscriber subscribeParameter = n2.subscribe("/bagdone", 1, optimizationCB);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
