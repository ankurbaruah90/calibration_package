//#include <iostream>
//#include <fstream>
//#include <ros/ros.h>
//#include <ros/package.h>
//#include <sensor_msgs/Image.h>
//#include <sensor_msgs/LaserScan.h>
//#include <sensor_msgs/PointCloud.h>
//#include <cv_bridge/cv_bridge.h>
//#include <laser_geometry/laser_geometry.h>
//#include <image_transport/image_transport.h>
//#include <image_geometry/pinhole_camera_model.h>
//#include "opencv2/core/core.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/core/core.hpp"
//#include "opencv2/features2d/features2d.hpp"
//#include "opencv2/highgui/highgui.hpp"
//#include "opencv2/imgproc/imgproc.hpp"
//#include "opencv2/video/tracking.hpp"
//#include <opencv/cv.h>
//#include <opencv/highgui.h>
//#include <pcl_ros/point_cloud.h>
//#include <pcl_conversions/pcl_conversions.h>
//#include <tf/transform_listener.h>
//#include <tf/transform_broadcaster.h>
//#include <geometry_msgs/TransformStamped.h>
//#include <sensor_msgs/PointCloud2.h>
//#include <tf2/convert.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf2_ros/buffer.h>
//#include <tf2/transform_datatypes.h>
//#include <tf2_ros/transform_listener.h>
//#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

//#define IMAGE_OUTPUT 0

//using namespace std;
//using namespace cv;

//int rows = 400, cols = 400;
//tf::TransformBroadcaster *br;
//ros::Publisher transformed_scan;
//int scanAccumulation = 0, windowSize = 0;
//laser_geometry::LaserProjection scanProjector;

//void imageCallBack(const sensor_msgs::ImageConstPtr &msg)
//{
//    msg->header.stamp = ros::Time::now();
//    std_msgs::Header h = msg->header;
//    cv_bridge::CvImagePtr cv_ptr;
//    try
//    {
//        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
//    }
//    catch (cv_bridge::Exception& e)
//    {
//        ROS_ERROR("cv_bridge exception: %s", e.what());
//        return;
//    }

//    cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2GRAY);

//    imshow("camera", cv_ptr->image);
//    waitKey(1);
//}

//void readScanCallBack(const sensor_msgs::LaserScanPtr &laserscan)
//{
//    tf::Matrix3x3 tf3d;
//    tf::Quaternion tfqt;
//    tf::Transform transform_laser;
//    tf3d.setEulerYPR(-8.42 * (M_PI/180),41.6 * (M_PI/180),16.2 * (M_PI/180));
//    tf3d.getRotation(tfqt);

//    transform_laser.setRotation(tfqt);
//    transform_laser.setOrigin(tf::Vector3(0.664, 0.441, 0.0127));
//    br->sendTransform(tf::StampedTransform(transform_laser.inverse(), laserscan->header.stamp, "laser", "camera_link"));

////    tf::TransformListener listener;
////    for (int i = 0; i < laserscan->ranges.size();i++)
////    {
////        double range = laserscan->ranges[i];
////        double angle  = laserscan->angle_min +(i * laserscan->angle_increment);

////        geometry_msgs::PointStamped laser_point, odom_point;

////        laser_point.header.frame_id = "laser";
////        laser_point.header.stamp = ros::Time();
////        laser_point.point.x = range*cos(angle) ;
////        laser_point.point.y = range*sin(angle) ;
////        laser_point.point.z = 0.0;

////        try
////        {
////            listener.waitForTransform("/camera_link", "/laser", laserscan->header.stamp, ros::Duration(0.2));
////            listener.transformPoint("laser", laser_point, odom_point);
////        }
////        catch(tf::TransformException& ex){
////            ROS_ERROR("%s", ex.what());
////        }


//        //Odom point x,y are in odom frame
//        // Copy or do your stuff
////    }

//#if IMAGE_OUTPUT
//    for(int i = 0; i < laserscan->ranges.size(); i++)
//        if(laserscan->ranges[i] < 1.5 || laserscan->ranges[i] > 4.0 || (laserscan->angle_min + (i)*laserscan->angle_increment) > 0.8)
//            laserscan->ranges[i] = 0;

//    scanAccumulation++;
//    std::vector<float> x, y;
//    cv::Mat A(rows, cols, CV_8UC1, cv::Scalar(0));
//    if (scanAccumulation > windowSize){
//        scanAccumulation = 0;

//        double maxrange = 5.0;
//        for(int i = 0; i < laserscan->ranges.size() - 1; i++)
//        {
//            if (laserscan->ranges.at(i) < maxrange && fabs(laserscan->ranges.at(i)) != 0.0)
//            {
//                double carvaluex1 = laserscan->ranges.at(i) * cos((laserscan->angle_min + (i)*laserscan->angle_increment));
//                double carvaluey1 =  laserscan->ranges.at(i) * sin((laserscan->angle_min + (i)*laserscan->angle_increment));

//                if(fabs(carvaluex1) != 0.0 || fabs(carvaluey1) != 0.0)
//                {
//                    x.push_back(carvaluex1);
//                    y.push_back(carvaluey1);
//                }
//            }
//        }

//        for(int i = 0; i < x.size(); i++)
//        {
//            int indexi = ((x[i] + maxrange)/(2 * maxrange)) * rows;
//            int indexj = ((y[i] + maxrange)/( 2 * maxrange))*cols;
//            if(indexi > rows)
//                indexi = rows;
//            if(indexj > cols)
//                indexj = cols;
//            A.at<uchar>(indexj,indexi) = 255;
//        }

//        x.clear();
//        y.clear();
//        imshow("laser", A);
//        waitKey(1);
//    }
//#endif

//}

//int main(int argc, char *argv[])
//{
//    ros::init(argc, argv, "project_laser_node");
//    ROS_INFO("Started Node");
//    ros::NodeHandle nh;
//    ros::Rate rate(20);
//    tf::TransformBroadcaster br_val;
//    br = &br_val;
//    ros::Subscriber imageSubscriber = nh.subscribe("/camera/rgb/image_color", 1, imageCallBack);
//    ros::Subscriber laserScanSubscriber = nh.subscribe("/scan", 1, readScanCallBack);
//    string path = ros::package::getPath("calibration_package");
//    system((path + "/launchBag.sh &").c_str());

//    while(ros::ok()){
//        ros::spinOnce();
//        rate.sleep();
//    }

//    return 0;
//}

#include <iostream>
#include <string>
#include <ros/ros.h>
#include <ros/package.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>

void readScanCallBack(const sensor_msgs::LaserScanPtr &laserscan)
{
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    geometry_msgs::TransformStamped transformStamped;
    try{
      transformStamped = tfBuffer.lookupTransform("base_link", "laser",
                               ros::Time(0), ros::Duration(0.2));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(0.2).sleep();
    }

    std::cout << "X " << transformStamped.transform.translation.x << std::endl;
    std::cout << "Y " << transformStamped.transform.translation.y << std::endl;
    std::cout << "Z " << transformStamped.transform.translation.z << std::endl;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "my_tf2_listener");

  ros::NodeHandle node;
  ros::Subscriber laserScanSubscriber = node.subscribe("/scan", 1, readScanCallBack);

  std::string path = ros::package::getPath("calibration_package");
  system((path + "/launchBag.sh &").c_str());

  ros::Rate rate(10.0);
  while (node.ok()){
    ros::spinOnce();
    rate.sleep();
  }
  return 0;
}


