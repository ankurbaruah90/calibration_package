#include "ros/ros.h"
#include "ros/console.h"
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <algorithm>
#include <math.h>
#include "tf/transform_datatypes.h"
#include "geometry_msgs/Quaternion.h"
#include "geometry_msgs/Pose2D.h"
#include "tf/LinearMath/Matrix3x3.h"
#include "tf/transform_listener.h"
#include <fstream>
#include <gsl/gsl_rng.h>
#include <string>
#include <time.h>

using namespace std;

struct tm tstruct;
char plot[100];
time_t now;
vector < vector <double> > log_data;

//void getCommFeedback(const purepursuit_autonav::communication::ConstPtr& feedback_ptr)
//{

//}

void getCorrectedOmega()
{

}

void getCmdVel()
{

}

void getFineTune()
{

}

void plotLog(const std_msgs::BoolPtr msg)
{
        now = time(0);
        tstruct = *localtime(&now);
        strftime(plot, sizeof(plot), "/home/ankur/Desktop/Logs/plot-%Y-%m-%d-%H-%M-%S", &tstruct);
        strcat(plot, ".csv");
            
	std::ofstream dataLog;
        dataLog.open(plot, std::ofstream::out | std::ofstream::app);
        //dataLog << setA[i].x << "," << setA[i].y << ","
        //        << setA[i].radian << "\t" << "," << setB[j].x << ","
        //        << setB[j].y << "," << setB[j].radian << "\n" ;
        dataLog.close();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "plot_graph");
    ros::NodeHandle nh;
//    ros::Subscriber subscribe_comm_feedback = nh.subscribe("/comm_feedback", 1, getCommFeedback);
//    ros::Subscriber subscribe_corrected_omega = nh.subscribe("/corrected_omega", 1, getCorrectedOmega);
//    ros::Subscriber subscribe_cmd_vel1 = nh.subscribe("/cmd_vel1", 1, getCmdVel);
//    ros::Subscriber subscribe_cmd_vel_finetune = nh.subscribe("/cmd_vel_finetune", 1, getFineTune);

    ros::Rate rate(20);
    while(ros::ok())
    {
        ros::spinOnce();
	rate.sleep();
    }
    return 0;
}
