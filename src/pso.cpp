#include <iostream>
#include <pso.h>
#include <ros/ros.h>
#include <time.h>
#include <fstream>
#include <string.h>
#include <std_msgs/Float32.h>
#include <calibration_package/Particle.h>

struct tm tstruct;
char plot[100];
time_t now;
ros::NodeHandle *n_;
ros::Publisher publish_particles;

float PSO::getScore(std::vector<float> values)
{

    // do something here and return the fitness score
    calibration_package::Particle particle_array;

    for (std::vector <float>::iterator it = values.begin(); it != values.end(); ++it)
        particle_array.param.push_back(*it);
    publish_particles.publish(particle_array);
    ROS_INFO("Published particles!");

    /// -----
    /// -----Generate particles
    /// -----Publish particles to calibrate node
    /// -----Launch Bag file
    /// -----Calculate score in calibrate file
    /// -----Wait for message (score) in getScore funciton
    /// -----

//    boost::shared_ptr<std_msgs::Float32 const> msg(new std_msgs::Float32);
//    msg = ros::topic::waitForMessage <std_msgs::Float32> ("/score", *n_, ros::Duration(10.0));

    boost::shared_ptr<std_msgs::Float32 const> msg(ros::topic::waitForMessage <std_msgs::Float32> ("/score", *n_, ros::Duration(10.0)));

    if (msg == NULL)
        return FLT_MAX;
    else
    {
        std::cout << "Error " << msg->data << std::endl;
        return msg->data;
    }
}

int main(int argc, char **argv)
{
//    now = time(0);
//    tstruct = *localtime(&now);
//    strftime(plot, sizeof(plot), "/home/ankur/Desktop/qt_dev/pso/log/PSO-SVM-%Y-%m-%d-%H-%M-%S", &tstruct);
//    strcat(plot, ".csv");

    ros::init(argc, argv, "optimize_node");
    ROS_INFO("Started Search Optimization Node");
    n_ = new ros::NodeHandle;
    publish_particles = n_->advertise<calibration_package::Particle>("particle_parameters", 1);

    PSO search;
    std::vector <float> parameters;
    parameters = search.getOptimal();

    for (std::vector <float>::iterator it = parameters.begin(); it != parameters.end(); ++it)
        std::cout << "Optimal Parameters " << *it << std::endl;

    delete n_;
    return 0;
}
