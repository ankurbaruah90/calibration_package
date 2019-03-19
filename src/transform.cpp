#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <opencv/cv.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <pcl_ros/point_cloud.h>
#include "tf/transform_listener.h"
#include <calibration_package/Particle.h>

///----------------------------------

//#include <fstream>
//#include <string.h>

//struct tm tstruct;
//char plot[100];
//time_t now;

///----------------------------------

using namespace std;
using namespace cv;

double cost = FLT_MAX;
ros::NodeHandle *n_;
ros::Publisher publish_score;
double estimated_x = 0, estimated_y = 0, estimated_z = 0;

void laserscanCB(const pcl::PointCloud <pcl::PointXYZ>::Ptr plane_points)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    bool transform_read = true;
    try
    {
        ros::Time t = ros::Time(0);
        listener.waitForTransform("/camera_optical_centre", "/checker_board", t, ros::Duration(0.5));
        listener.lookupTransform("/camera_optical_centre", "/checker_board", t, transform);
//        listener.setExtrapolationLimit(ros::Duration(0.1));
    }
    catch (tf::TransformException &ex)
    {
        ROS_ERROR("%s",ex.what());
        transform_read = false;
        ros::Duration(0.1).sleep();
    }

    if(transform_read)
    {
        ///----------------------------------------------------------------------///
        ///----Finding the equation of the z = 0 plane w.r.t. camera frame-------///
        /// ---------------------------------------------------------------------///

        tf::Point position = transform.getOrigin();
        tf::Quaternion rotQuaternion = transform.getRotation();
        tf::Matrix3x3 m(rotQuaternion);

        Mat tvec = cv::Mat(1, 3, CV_32F);
        Mat rvec = cv::Mat(3, 3, CV_32F);

        tvec.at<float>(0,0) = position.getX(); tvec.at<float>(0,1) = position.getY(); tvec.at<float>(0,2) = position.getZ();
        rvec.at<float>(0,0) = m.getColumn(0)[0]; rvec.at<float>(0,1) = m.getColumn(1)[0]; rvec.at<float>(0,2) = m.getColumn(2)[0];
        rvec.at<float>(1,0) = m.getColumn(0)[1]; rvec.at<float>(1,1) = m.getColumn(1)[1]; rvec.at<float>(1,2) = m.getColumn(2)[1];
        rvec.at<float>(2,0) = m.getColumn(0)[2]; rvec.at<float>(2,1) = m.getColumn(1)[2]; rvec.at<float>(2,2) = m.getColumn(2)[2];

        Mat rxt = cv::Mat(1, 3, CV_32F);
        rxt = -1 * tvec * rvec;

        Mat inverse_transpose = cv::Mat(4, 4, CV_32F);
        inverse_transpose.at<float>(0,0) = rvec.at<float>(0,0); inverse_transpose.at<float>(0,1) = rvec.at<float>(0,1); inverse_transpose.at<float>(0,2) = rvec.at<float>(0,2); inverse_transpose.at<float>(0,3) = 0;
        inverse_transpose.at<float>(1,0) = rvec.at<float>(1,0); inverse_transpose.at<float>(1,1) = rvec.at<float>(1,1); inverse_transpose.at<float>(1,2) = rvec.at<float>(1,2); inverse_transpose.at<float>(1,3) = 0;
        inverse_transpose.at<float>(2,0) = rvec.at<float>(2,0); inverse_transpose.at<float>(2,1) = rvec.at<float>(2,1); inverse_transpose.at<float>(2,2) = rvec.at<float>(2,2); inverse_transpose.at<float>(2,3) = 0;
        inverse_transpose.at<float>(3,0) = rxt.at<float>(0,0); inverse_transpose.at<float>(3,1) = rxt.at<float>(0,1); inverse_transpose.at<float>(3,2) = rxt.at<float>(0,2); inverse_transpose.at<float>(3,3) = 1;

        Mat z0 = cv::Mat(4, 1, CV_32F);
        z0.at<float>(0,0) = 0;
        z0.at<float>(1,0) = 0;
        z0.at<float>(2,0) = 1;
        z0.at<float>(3,0) = 0;

        ///----Finding the equation of z = 0 plane w.r.t. the camera optical frame----///
        Mat plane_eq = cv::Mat(4, 1, CV_32F);
        plane_eq = inverse_transpose * z0;

        cout << "Plane Equation\n" << plane_eq << endl;


        ///-------------------------------------------------------///
        ///------Transform to camera frame using the random-------///
        ///---------------generated particle values---------------///
        /// ------------------------and---------------------------///
        ///----converting pcl::pointxyz to vector <vector3>-------///
        /// ------------------------------------------------------///
        ///-------------------------------------------------------///
        ///----Minimizing the cost function in which the----------///
        ///----laser points satisfy the plane equation------------///
        /// ------------------------------------------------------///

        BOOST_FOREACH (const pcl::PointXYZ& pt, plane_points->points){
            tf::Vector3 temp(pt.x, pt.y, pt.z);
            cost += fabs(plane_eq.at<float>(0,0)*(temp.x() + estimated_x) +
                    plane_eq.at<float>(1,0)*(temp.y() + estimated_y) +
                    plane_eq.at<float>(2,0)*(temp.z() + estimated_z) +
                    plane_eq.at<float>(3,0));
        }
        cout << "Cost " << cost << endl;
    }
}

void particleCB(const calibration_package::ParticlePtr &msg)
{
    ROS_INFO("Received particles!!");

    for (int i = 0; i < msg->param.size(); i++)
        cout << msg->param[i] << endl;

    estimated_x = msg->param[0];
    estimated_y = msg->param[1];
    estimated_z = msg->param[2];

    std::string path = ros::package::getPath("calibration_package");
    system((path + "/launchBag.sh &").c_str());


    ///------------------------------------------

//    std::ofstream dataLog;
//    dataLog.open(plot, std::ofstream::out | std::ofstream::app);
//    dataLog << x << ", " << y << ", " << cost << "\n";
//    dataLog.close();

    ///------------------------------------------
}

void bagCompletionCB(const std_msgs::BoolPtr msg)
{
    std_msgs::Float32 error_score;
    if(msg->data)
    {
        error_score.data = cost;
        publish_score.publish(error_score);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibrate_node");
    ROS_INFO("Started calibration node");
    n_ = new ros::NodeHandle;
    ros::Rate rate(1);
    publish_score = n_->advertise<std_msgs::Float32>("score", 1);
    ros::Subscriber laserScanSubscriber = n_->subscribe("/calibration_plane", 1, laserscanCB);
    ros::Subscriber subscribeParticle = n_->subscribe("/particle_parameters", 1, particleCB);
    ros::Subscriber subscribeBagCompletionFlag = n_->subscribe("/bagdone", 1, bagCompletionCB);

    ///------------------------------------------

//    now = time(0);
//    tstruct = *localtime(&now);
//    strftime(plot, sizeof(plot), "/home/ankur/Desktop/PSO-%Y-%m-%d-%H-%M-%S", &tstruct);
//    strcat(plot, ".csv");

    ///------------------------------------------

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    delete n_;
    return 0;
}
