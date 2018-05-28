#include <iostream>
#include <ros/ros.h>
#include <ros/package.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <laser_geometry/laser_geometry.h>
#include <image_geometry/pinhole_camera_model.h>
#include "opencv2/core/core.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/video/tracking.hpp"
#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <tf/transform_listener.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/TransformStamped.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>
#include <pcl_ros/transforms.h>
#include <pcl/PCLPointCloud2.h>

#define IMAGE_OUTPUT 0
#define TRANSFORM_TO_BASE_LINK 1
#define WRITE 0

using namespace std;
using namespace cv;

ros::Publisher pub, pub_sensor;
int rows = 400, cols = 400;
int scanAccumulation = 0, windowSize = 0;
laser_geometry::LaserProjection scanProjector;

void readScanCallBack2(const sensor_msgs::LaserScanPtr &laserscan)
{
//    std::ofstream laserLog;
//    std_msgs::Header h = laserscan->header;
//    laserLog.open("/home/ankur/Desktop/NewImage/data.txt", std::ofstream::out | std::ofstream::app);
//    laserLog << h.stamp.sec << "." << h.stamp.nsec << ", ";
//    laserLog << laserscan->angle_min << ", " << laserscan->angle_increment << ", " << laserscan->angle_max << ", ";
//    laserLog << "3, " << laserscan->ranges.size();
//    for(int i = 0; i < laserscan->ranges.size(); i++)
//        laserLog  << ", " << laserscan->ranges.at(i);
//    laserLog << endl;
//    laserLog.close();
}

void readScanCallBack(const sensor_msgs::LaserScanPtr &laserscan)
{
    bool transform_read = true;
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformStamped;
    try{
        transformStamped = tfBuffer.lookupTransform("base_link", "laser", ros::Time(0), ros::Duration(0.2));
    }
    catch (tf2::TransformException &ex) {
        transform_read = false;
        ROS_WARN("%s",ex.what());
        ros::Duration(0.2).sleep();
    }

    if(transform_read){
        std::ofstream laserLog;
        std_msgs::Header h = laserscan->header;
        laserscan->header.stamp = ros::Time(0);

//        for(int i = 0; i < laserscan->ranges.size(); i++)
//            if(laserscan->ranges[i] < 1.5 || laserscan->ranges[i] > 4.0 || (laserscan->angle_min + (i)*laserscan->angle_increment) > 0.8)
//                laserscan->ranges[i] = 0;

#if TRANSFORM_TO_BASE_LINK
        sensor_msgs::PointCloud2 cloud_in, cloud_out;
        cloud_in.header.frame_id = laserscan->header.frame_id;
        scanProjector.projectLaser(*laserscan, cloud_in);
        cloud_in.header.stamp = laserscan->header.stamp;

        tf2::doTransform(cloud_in, cloud_out, transformStamped);
        cloud_out.header.stamp = laserscan->header.stamp;
        cloud_out.header.frame_id = "base_link";
        pub_sensor.publish(cloud_out);
#endif
{
#if WRITE
        laserLog.open("/home/ankur/Desktop/NewImage/data.txt", std::ofstream::out | std::ofstream::app);
        laserLog << h.stamp.sec << "." << h.stamp.nsec << ", ";
        laserLog << laserscan->angle_min << ", " << laserscan->angle_increment << ", " << laserscan->angle_max << ", ";
        laserLog << "3, " << laserscan->ranges.size();
        for(int i = 0; i < laserscan->ranges.size(); i++)
            laserLog  << ", " << laserscan->ranges.at(i);
        laserLog << endl;
        laserLog.close();
#endif

        for(int i = 0; i < laserscan->ranges.size(); i++)
            if(laserscan->ranges[i] < 1.5 || laserscan->ranges[i] > 4.0 || (laserscan->angle_min + (i)*laserscan->angle_increment) > 0.8)
                laserscan->ranges[i] = 0;

#if IMAGE_OUTPUT
        scanAccumulation++;
        std::vector<float> x, y;
        cv::Mat A(rows, cols, CV_8UC1, cv::Scalar(0));
        if (scanAccumulation > windowSize){
            scanAccumulation = 0;

            double maxrange = 5.0;
            for(int i = 0; i < laserscan->ranges.size() - 1; i++)
            {
                if (laserscan->ranges.at(i) < maxrange && fabs(laserscan->ranges.at(i)) != 0.0)
                {
                    double carvaluex1 = laserscan->ranges.at(i) * cos((laserscan->angle_min + (i)*laserscan->angle_increment));
                    double carvaluey1 =  laserscan->ranges.at(i) * sin((laserscan->angle_min + (i)*laserscan->angle_increment));

                    if(fabs(carvaluex1) != 0.0 || fabs(carvaluey1) != 0.0)
                    {
                        x.push_back(carvaluex1);
                        y.push_back(carvaluey1);
                    }
                }
            }

            for(int i = 0; i < x.size(); i++)
            {
                int indexi = ((x[i] + maxrange)/(2 * maxrange)) * rows;
                int indexj = ((y[i] + maxrange)/( 2 * maxrange))*cols;
                if(indexi > rows)
                    indexi = rows;
                if(indexj > cols)
                    indexj = cols;
                A.at<uchar>(indexj,indexi) = 255;
            }

            x.clear();
            y.clear();
            imshow("image", A);
            waitKey(1);
        }
#endif
}
        //sensor_msgs::laserScan to sensor_msgs::pointcloud2-------------------------------------------
        sensor_msgs::PointCloud2 cloud;
        cloud.header.frame_id = laserscan->header.frame_id;
        scanProjector.projectLaser(*laserscan, cloud);
        cloud.header.stamp = laserscan->header.stamp;

        //sensor_msgs::pointcloud2 to pcl::pointcloud2-------------------------------------------------
        pcl::PCLPointCloud2 pcl_cloud;
        pcl_conversions::toPCL(cloud, pcl_cloud);

        //pcl::PointCloud2 to pcl::PointCloud----------------------------------------------------------
        std::vector <int> inliers;
        pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr final_points(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_cloud,*transformed_cloud);

        //using pcl racsac for line fitting------------------------------------------------------------
        pcl::SampleConsensusModelLine <pcl::PointXYZ>::Ptr lineModel(new pcl::SampleConsensusModelLine <pcl::PointXYZ> (transformed_cloud));
        pcl::RandomSampleConsensus <pcl::PointXYZ> ransac(lineModel);
        ransac.setDistanceThreshold(0.1);
        ransac.computeModel();
        ransac.getInliers(inliers);

        pcl::copyPointCloud(*transformed_cloud, inliers, *final_points);
        pub.publish(final_points);

        //    ROS_INFO("Cloud: width = %d, height = %d\n", transformed_cloud->width, transformed_cloud->height);
        //    ROS_INFO("Final: width = %d, height = %d\n", final_points->width, final_points->height);
        //    BOOST_FOREACH (const pcl::PointXYZ& pt, final_points->points)
        //      ROS_INFO("\t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

        sensor_msgs::PointCloud2 final_points_laser;
        pcl::toROSMsg(*final_points, final_points_laser);
        //    pub_sensor.publish(final_points_laser);
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "laser_calibration_node");
    ROS_INFO("Started Node");

    ros::NodeHandle node;
    ros::Subscriber laserScanSubscriber = node.subscribe("/scan_out", 1, readScanCallBack);
    ros::Subscriber laserScanSubscriber2 = node.subscribe("/calibration_plane_laser", 1, readScanCallBack2);
    pub = node.advertise < pcl::PointCloud <pcl::PointXYZ> > ("calibration_plane", 1);
    pub_sensor = node.advertise < sensor_msgs::PointCloud2 > ("calibration_plane_sensor", 1);

    std::string path = ros::package::getPath("calibration_package");
    system((path + "/launchBag.sh &").c_str());

    ros::Rate rate(10.0);
    while (node.ok()){
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
