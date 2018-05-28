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
#include <pso.cpp>

#define OPTIMIZE 1
#define BREGMAN 0

using namespace std;
using namespace cv;

std::map <vector <tf::Vector3>, tf::Vector3> pointPoseMap;

#if BREGMAN
vector < vector <float> > P0, B0, tmp;
void PSO::initialize()
{
    /* ------------ Initialize Swarm  -------------  */
    localBestFitness.resize(SWARM_SIZE);
    currentFitness.resize(SWARM_SIZE);
    localBestPosition.resize(DIM);
    globalBestPosition.resize(DIM);
    currentPosition.resize(DIM);
    velocity.resize(DIM);
    P0.resize(DIM);
    B0.resize(DIM);
    tmp.resize(DIM);
    for (int i = 0 ; i < DIM; i++)
    {
        currentPosition[i].resize(SWARM_SIZE);
        localBestPosition[i].resize(SWARM_SIZE);
        globalBestPosition[i].resize(SWARM_SIZE);
        velocity[i].resize(SWARM_SIZE);
        P0[i].resize(SWARM_SIZE);
        B0[i].resize(SWARM_SIZE);
        tmp[i].resize(SWARM_SIZE);
        for (int j = 0; j < SWARM_SIZE; j++)
        {
#if FINETUNE
            std::ifstream file("/home/ankur/catkin_ws/src/navigation/log/examples/parameters.yaml");
            std::string str, value;
            std::vector <std::string> file_contents;
            while (std::getline(std::getline(file, str, ':'), value, '\n'))
                file_contents.push_back(value);
            double generateRandom = 0.2 * (gsl_rng_uniform(r) - 0.5);
            currentPosition[i][j] = generateRandom + atof(file_contents[i].c_str());
            if(currentPosition[i][j] < 0.0)
                currentPosition[i][j] = 0.001;
//            cout << "currentValue: " << currentPosition[i][j] << " originalValue: " << atof(file_contents[i].c_str()) << " difference: " << (currentPosition[i][j] - atof(file_contents[i].c_str())) << "\n";
            file.close();
#else
            currentPosition[i][j] = SPAN * (gsl_rng_uniform(r) - 0.5);         // random initial swarm positions, positive values
            P0[i][j] = currentPosition[i][j];
#endif
            velocity[i][j] = 0.05 * (gsl_rng_uniform(r));             // random initial swarm velocities
        }
    }
    localBestPosition = currentPosition;
}
#endif

void PSO::updateVelocityAndPosition()
{
    /* ------- update velocity and position ------ */
    for (int i = 0; i < DIM; i++)
        for (int j = 0; j < SWARM_SIZE; j++)
        {
            random1 = gsl_rng_uniform(r);
            random2 = gsl_rng_uniform(r);
            velocity[i][j] =  inertia * velocity[i][j] + c1 * (random1 * (localBestPosition[i][j] - currentPosition[i][j])) + c2 * (random2 * (globalBestPosition[i][j] - currentPosition[i][j]));
            currentPosition[i][j] = currentPosition[i][j] + velocity[i][j];
        }
}

void PSO::getFitness(){
    for (int i = 0; i < SWARM_SIZE; i++)
    {
        Mat tmpMatrix1(Size(3, 3), CV_32F);
        Mat tmpMatrix2(Size(3, 3), CV_32F);

        for (int j = 0; j < currentPosition.size(); j++)
            cout << "Particles " << currentPosition[j][i] << endl;

        tmpMatrix1.at<float>(0, 0) = currentPosition[3][i];
        tmpMatrix1.at<float>(0, 1) = currentPosition[4][i];
        tmpMatrix1.at<float>(0, 2) = currentPosition[5][i];
        tmpMatrix1.at<float>(1, 0) = currentPosition[6][i];
        tmpMatrix1.at<float>(1, 1) = currentPosition[7][i];
        tmpMatrix1.at<float>(1, 2) = currentPosition[8][i];
        tmpMatrix1.at<float>(2, 0) = currentPosition[9][i];
        tmpMatrix1.at<float>(2, 1) = currentPosition[10][i];
        tmpMatrix1.at<float>(2, 2) = currentPosition[11][i];

        tmpMatrix2 = tmpMatrix1.inv();

        float error = 0.0;
        for (std::map <vector <tf::Vector3>, tf::Vector3>::iterator it = pointPoseMap.begin(); it != pointPoseMap.end(); ++it){
            float local_error = 0.0;
            for (std::vector <tf::Vector3>::const_iterator itr  = it->first.begin(); itr != it->first.end(); ++itr){

                /*------------------cost function------------------*/
                tf::Vector3 tmp = *itr;

                //translation of point on camera reference frame----------------------------------
                float tx = tmp.x() - currentPosition[0][i];
                float ty = tmp.y() - currentPosition[1][i];
                float tz = tmp.z() - currentPosition[2][i];

                //rotation of point on camera reference frame-------------------------------------
                float p1 = tmpMatrix2.at<float>(0, 0) * tx + tmpMatrix2.at<float>(0, 1) * ty + tmpMatrix2.at<float>(0, 2) * tz;
                float p2 = tmpMatrix2.at<float>(1, 0) * tx + tmpMatrix2.at<float>(1, 1) * ty + tmpMatrix2.at<float>(1, 2) * tz;
                float p3 = tmpMatrix2.at<float>(2, 0) * tx + tmpMatrix2.at<float>(2, 1) * ty + tmpMatrix2.at<float>(2, 2) * tz;

                //multiplication of unit normal to p1, p2 & p3
                float magnitude = sqrt(pow((it->second).x(), 2) + pow((it->second).y(), 2) + pow((it->second).z(), 2));
                float distance = (((it->second).x() * p1 + (it->second).y() * p2 + (it->second).z() * p3)/magnitude) - magnitude;
                local_error += pow(distance, 2);

//                cout << "Magnitude " << magnitude << " Distance " << distance << " Local Error " << local_error << endl;
            }
            error += local_error;
//            cout << "Total Local Error " << error << endl;
        }

        // Calculating Error
        currentFitness[i] = error;

#if BREGMAN

        for (int val = 3; val < 12; val++){
            tmp[val][i] = pow((currentPosition[val][i] - P0[val][i] + B0[val][i]),2);
            P0[val][i] = (tmp[val][i] + B0[val][i]) / pow((tmp[val][i] + B0[val][i]),2);
            B0[val][i] += tmp[val][i] - P0[val][i];
        }

        ///Exterior Penalty - Quadratic Loss Function - orthonormality of rotation matrix---------
        float val1 = sqrt(pow(P0[3][i],2) + pow(P0[6][i],2) + pow(P0[9][i],2));
        float val2 = sqrt(pow(P0[4][i],2) + pow(P0[7][i],2) + pow(P0[10][i],2));
        float val3 = sqrt(pow(P0[5][i],2) + pow(P0[8][i],2) + pow(P0[11][i],2));

        currentFitness[i] += penalty*(pow((val1 - 1), 2) + pow((val2 - 1), 2) + pow((val3 - 1), 2));

#else
        ///Exterior Penalty - Quadratic Loss Function - orthonormality of rotation matrix---------
        float val1 = (pow(currentPosition[3][i],2) + pow(currentPosition[6][i],2) + pow(currentPosition[9][i],2));
        float val2 = (pow(currentPosition[4][i],2) + pow(currentPosition[7][i],2) + pow(currentPosition[10][i],2));
        float val3 = (pow(currentPosition[5][i],2) + pow(currentPosition[8][i],2) + pow(currentPosition[11][i],2));
        currentFitness[i] += penalty*(pow((1 - val1), 2) + pow((1 - val2), 2) + pow((1 - val3), 2));
#endif

        cout << "Fitness " << currentFitness[i] << "\n\n";

#if LOG

        std::ofstream dataLog;
        dataLog.open(log, std::ofstream::out | std::ofstream::app);
        dataLog << i << ",";
        for (int j = 0; j < currentPosition.size(); j++)
            dataLog << currentPosition[j][i] << ",";
        dataLog << currentFitness[i] << "\t" << ",";
        dataLog.close();

#endif

    }
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
        pointPoseMap.insert(std::pair<vector <tf::Vector3>, tf::Vector3>(line_points, normal));
        line_points.clear();
    }
}

void optimizationCB(const std_msgs::BoolPtr msg)
{
    if(msg->data)
    {
#if OPTIMIZE

//        for (std::map <vector <tf::Vector3>, tf::Vector3>::iterator it = pointPoseMap.begin(); it != pointPoseMap.end(); ++it)
//            for (std::vector <tf::Vector3>::const_iterator itr  = it->first.begin(); itr != it->first.end(); ++itr){
//                tf::Vector3 tmp = *itr;
//                ROS_INFO("Points (%f, %f, %f) - (%f, %f, %f)", it->second.x(), it->second.y(), it->second.z(), tmp.x(), tmp.y(), tmp.z());
//            }

        ROS_INFO("Started Optimization");
        PSO optimization;
        vector <float> optimalValues;
        optimalValues = optimization.getOptimal();

        //translation vector and rotation matrix (find inverse)----------------------------------------
        ROS_INFO("Finished Calibrating...\nOptimal Values are...");
        for (int i = 0; i < optimalValues.size(); i++)
            ROS_INFO("Parameter %d -- %f", i, optimalValues[i]);

        ROS_INFO("\nRotation: \n %f %f %f \n %f %f %f \n %f %f %f",
                 optimalValues[3], optimalValues[4], optimalValues[5],
                optimalValues[6], optimalValues[7], optimalValues[8],
                optimalValues[9], optimalValues[10], optimalValues[11]);
        ROS_INFO("\nTranslation: \n %f %f %f", optimalValues[0], optimalValues[1], optimalValues[2]);

#endif
        exit(0);
    }
}

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
