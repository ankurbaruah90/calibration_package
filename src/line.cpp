#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include "opencv2/highgui/highgui.hpp"
#include <opencv/cv.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_line.h>

#define IMAGE_OUTPUT 1
#define WRITE 0

using namespace std;
using namespace cv;

ros::Publisher pub, pub_sensor;
int rows = 400, cols = 400;
int scanAccumulation = 0, windowSize = 0;
laser_geometry::LaserProjection scanProjector;

void readScanCallBack(const sensor_msgs::LaserScanPtr &laserscan)
{
    std_msgs::Header h = laserscan->header;
    laserscan->header.stamp = ros::Time(0);

    for(int i = 0; i < laserscan->ranges.size(); i++)
        if(laserscan->ranges[i] < 1.5 || laserscan->ranges[i] > 4.0 || (laserscan->angle_min + (i)*laserscan->angle_increment) > 0.8)
            laserscan->ranges[i] = 0;

#if WRITE
    std::ofstream laserLog;
    laserLog.open("/home/ankur/Desktop/NewImage/data.txt", std::ofstream::out | std::ofstream::app);
    laserLog << h.stamp.sec << "." << h.stamp.nsec << ", ";
    laserLog << laserscan->angle_min << ", " << laserscan->angle_increment << ", " << laserscan->angle_max << ", ";
    laserLog << "3, " << laserscan->ranges.size();
    for(int i = 0; i < laserscan->ranges.size(); i++)
        laserLog  << ", " << laserscan->ranges.at(i);
    laserLog << endl;
    laserLog.close();
#endif

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

    //laser data in pcl pointcloud
    pcl::copyPointCloud(*transformed_cloud, inliers, *final_points);
    pub.publish(final_points);

    ROS_INFO("Cloud: width = %d, height = %d\n", transformed_cloud->width, transformed_cloud->height);
    ROS_INFO("Final: width = %d, height = %d\n", final_points->width, final_points->height);
    BOOST_FOREACH (const pcl::PointXYZ& pt, final_points->points)
            ROS_INFO("Line Points \t(%f, %f, %f)\n", pt.x, pt.y, pt.z);

    //laser data in sensor_msgs point cloud 2
    sensor_msgs::PointCloud2 final_points_laser;
    pcl::toROSMsg(*final_points, final_points_laser);
    pub_sensor.publish(final_points_laser);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "line_node");
    ROS_INFO("Started Laser Line Extraction Node");

    ros::NodeHandle node;
    ros::Subscriber laserScanSubscriber = node.subscribe("/r2000_driver_node/scan", 1, readScanCallBack);
    pub = node.advertise < pcl::PointCloud <pcl::PointXYZ> > ("calibration_plane", 1);
    pub_sensor = node.advertise < sensor_msgs::PointCloud2 > ("calibration_plane_sensor", 1);

    ros::Rate rate(20.0);
    while (node.ok()){
        ros::spinOnce();
        rate.sleep();
    }
    return 0;
}
