#include <ros/ros.h>
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
#include <pcl_ros/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>
#include <opencv/highgui.h>
#include "tf/transform_listener.h"
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Pose2D.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Vector3.h>
#include "tf/LinearMath/Matrix3x3.h"
#include <pso.h>

using namespace std;
using namespace cv;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "transform_node");
    ROS_INFO("Started transform node");
    ros::NodeHandle n1, n2;
    ros::Rate rate(40);
    ros::Subscriber laserScanSubscriber = n1.subscribe("/calibration_plane", 1, laserscanCB);
    ros::Subscriber subscribeParameter = n2.subscribe("/bagdone", 1, optimizationCB);

    while(ros::ok())
    {
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
