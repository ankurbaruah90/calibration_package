#include <iostream>
#include <pso.h>
#include <ros/ros.h>
#include <time.h>
#include <fstream>
#include <string.h>
#include <std_msgs/Float32.h>

struct tm tstruct;
char plot[100];
time_t now;
ros::NodeHandle n;

float PSO::getScore(std::vector<float> values)
{

    // do something here and return the fitness score

    /// -----
    /// -----Generate particles
    /// -----Publish particles to calibrate node
    /// -----Launch Bag file
    /// -----Calculate score in calibrate file
    /// -----Wait for message (score) in getScore funciton
    /// -----

    std_msgs::Float32::ConstPtr msg = ros::topic::waitForMessage <std_msgs::Float32> ("/Score", n);
    float cost = abs(msg->data);

    return cost;
}

int main(int argc, char **argv)
{
//    now = time(0);
//    tstruct = *localtime(&now);
//    strftime(plot, sizeof(plot), "/home/ankur/Desktop/qt_dev/pso/log/PSO-SVM-%Y-%m-%d-%H-%M-%S", &tstruct);
//    strcat(plot, ".csv");

    ros::init(argc, argv, "calibrate_node");
    ROS_INFO("Started Search Optimization Node");

    PSO search;
    std::vector <float> parameters;
    parameters = search.getOptimal();

    for (std::vector <float>::iterator it = parameters.begin(); it != parameters.end(); ++it)
        std::cout << "Optimal Parameters " << *it << std::endl;

    return 0;
}
