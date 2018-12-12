#include <iostream>
#include <ros/ros.h>
#include <opencv/cv.h>
#include <pcl_ros/point_cloud.h>
#include <pso.h>
#include "tf/transform_listener.h"

using namespace std;
using namespace cv;

float PSO::getScore(std::vector<float> values)
{

    return 0.0;
}

void laserscanCB(const pcl::PointCloud <pcl::PointXYZ>::Ptr plane_points)
{
    tf::TransformListener listener;
    tf::StampedTransform transform;
    bool transform_read = true;
    try
    {
        ros::Time t = ros::Time(0);
        listener.waitForTransform("/camera", "/checker_board", t, ros::Duration(0.5));
        listener.lookupTransform("/camera", "/checker_board", t, transform);
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
        //calculating normal to calibration plane------------------------------------------------------
        tf::Point position = transform.getOrigin();
        tf::Quaternion rotQuaternion = transform.getRotation();
        tf::Matrix3x3 m(rotQuaternion);
        float magnitude = m.getColumn(2).x() * position.x() + m.getColumn(2).y() * position.y() + m.getColumn(2).z() * position.z();
        tf::Vector3 normal = -magnitude * m.getColumn(2);
//        ROS_INFO("Position %f -- %f -- %f", position.x(), position.y(), position.z());
//        ROS_INFO("Rotation Mat %f -- %f -- %f", m.getColumn(2).x(), m.getColumn(2).y(), m.getColumn(2).z());

        //converting pcl::pointxyz to vector <vector3>-------------------------------------------------
        vector <tf::Vector3> line_points;
        BOOST_FOREACH (const pcl::PointXYZ& pt, plane_points->points){
            tf::Vector3 temp(pt.x, pt.y, pt.z);;
//                    ROS_INFO("Points (%f, %f, %f) - (%f, %f, %f)", normal.x(), normal.y(), normal.z(), temp.x(), temp.y(), temp.z());
            line_points.push_back(temp);
        }

        //mapping vector of points on calibration plane to their corresponding normal------------------
        std::map <vector <tf::Vector3>, tf::Vector3> pointPoseMap;
        pointPoseMap.insert(std::pair<vector <tf::Vector3>, tf::Vector3>(line_points, normal));
        line_points.clear();
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "calibrate_node");
    ROS_INFO("Started transform node");
    ros::NodeHandle n1, n2;
    ros::Rate rate(20);
    ros::Subscriber laserScanSubscriber = n1.subscribe("/calibration_plane", 1, laserscanCB);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
