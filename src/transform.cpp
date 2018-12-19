#include <iostream>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <pcl_ros/point_cloud.h>
#include "tf/transform_listener.h"
#include <calibration_package/Particle.h>

using namespace std;
using namespace cv;

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
        ///----converting pcl::pointxyz to vector <vector3>-------///
        /// ------------------------------------------------------///

        vector <tf::Vector3> line_points;
        BOOST_FOREACH (const pcl::PointXYZ& pt, plane_points->points){
            tf::Vector3 temp(pt.x, pt.y, pt.z);;
//                    ROS_INFO("Points (%f, %f, %f) - (%f, %f, %f)", normal.x(), normal.y(), normal.z(), temp.x(), temp.y(), temp.z());
            line_points.push_back(temp);
        }

        ///-------------------------------------------------------///
        ///----Minimizing the cost function in which the----------///
        ///----laser points satisfy the plane equation------------///
        /// ------------------------------------------------------///

//        std::map <vector <tf::Vector3>, tf::Vector3> pointPoseMap;
//        pointPoseMap.insert(std::pair<vector <tf::Vector3>, tf::Vector3>(line_points, normal));
//        line_points.clear();
    }
}

void particleCB(const calibration_package::ParticlePtr msg)
{
    for (int i = 0; i < msg->param.size(); i++)
        cout << msg->param[i] << endl;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibrate_node");
    ROS_INFO("Started calibration node");
    ros::NodeHandle n1;
    ros::Rate rate(20);
    ros::Subscriber laserScanSubscriber = n1.subscribe("/calibration_plane", 1, laserscanCB);
    ros::Subscriber subscribeParticle = n1.subscribe("/particles", 1, particleCB);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
