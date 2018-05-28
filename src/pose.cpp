#include <iostream>
#include <fstream>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
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
#include <tf/transform_broadcaster.h>

#define WRITE 0

using namespace std;
using namespace cv;

/*
    1. Laser scan frame
    2. Geometry sanity
    3. Check optimization, add orthonormality constraint
*/

int number = 0;
ros::Publisher pub;
static void meshgrid(const cv::Mat &xgv, const cv::Mat &ygv,
                     cv::Mat1i &X, cv::Mat1i &Y)
{
    cv::repeat(xgv.reshape(1,1), ygv.total(), 1, X);
    cv::repeat(ygv.reshape(1,1).t(), 1, xgv.total(), Y);
}

static void meshgridTest(const cv::Range &xgv, const cv::Range &ygv,
                         cv::Mat1i &X, cv::Mat1i &Y)
{
    std::vector<int> t_x, t_y;
    for (int i = xgv.start; i <= xgv.end; i++)
        t_x.push_back(i);
    for (int i = ygv.start; i <= ygv.end; i++)
        t_y.push_back(i);
    meshgrid(cv::Mat(t_x), cv::Mat(t_y), X, Y);
}

void imageCallBack(const sensor_msgs::ImageConstPtr &msg)
{
    std::ofstream timeLog;
    msg->header.stamp = ros::Time(0);
    std_msgs::Header h = msg->header;
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

//    cv::Mat img = cv_ptr->image;
    cvtColor(cv_ptr->image, cv_ptr->image, CV_RGB2GRAY);
    cv::imshow("image_raw", cv_ptr->image);
    cv::waitKey(1);

#if WRITE
    if(!(number++ % 10))
    {
        timeLog.open("/home/ankur/Desktop/NewImage/val.txt", std::ofstream::out | std::ofstream::app);
        timeLog << h.stamp.sec << "." << h.stamp.nsec << ", " << h.stamp.sec << ".0" << h.stamp.nsec << endl;
        imwrite(("/home/ankur/Desktop/NewImage/image" + to_string(number) + ".png").c_str(), cv_ptr->image);
        timeLog.close();
    }
#endif

    vector<Point2f> pointBuf;
    bool found = findChessboardCorners(cv_ptr->image, Size(5,3), pointBuf, CV_CALIB_CB_ADAPTIVE_THRESH);
    if (found){
        cornerSubPix(cv_ptr->image, pointBuf, Size(11, 11), Size(-1, -1),
            TermCriteria(CV_TERMCRIT_EPS + CV_TERMCRIT_ITER, 30, 0.1));

        drawChessboardCorners(cv_ptr->image, Size(5,3), Mat(pointBuf), found);

        cv::imshow("corners", cv_ptr->image);
        cv::waitKey(1);

        cout << "Top Left     X " << pointBuf[0].x << " Y " << pointBuf[0].y << endl;
        cout << "Top Right    X " << pointBuf[4].x << " Y " << pointBuf[4].y << endl;
        cout << "Bottom Left  X " << pointBuf[10].x << " Y " << pointBuf[10].y << endl;
        cout << "Bottom Right X " << pointBuf[14].x << " Y " << pointBuf[14].y << endl;
/*
        // to point2d
        vector<Point2d> pointBuf2d;
        vector<Point3d> boardPoints;
        for (size_t i = 0 ; i < pointBuf.size(); i++)
        {
            pointBuf2d.push_back(cv::Point2d((double)pointBuf[i].x, (double)pointBuf[i].y));
            boardPoints.push_back(cv::Point3d(0.0, 0.0, 0.0));
        }

        cv::Mat1i X, Y;
        meshgridTest(cv::Range(0,7), cv::Range(0, 5), X, Y);

        int count = 0;
        for (cv::Mat1i::iterator it = X.begin(); it != X.end(); ++it)
            boardPoints[count++].x = *it;
        count = 0;
        for (cv::Mat1i::iterator it = Y.begin(); it != Y.end(); ++it)
            boardPoints[count++].y = *it;

        Mat axis = (Mat_<double>(3, 3) << 3, 0, 0, 0, 3, 0, 0, 0, -3);
        Mat cameraMatrix = (Mat_<double>(3, 3) << 528.995513, 0, 303.036604, 0, 527.983687, 257.009383, 0, 0, 1);
        Mat distortionCoefficients = (Mat_<double>(1, 5) << 0.131585, -0.191955, -0.005917, -0.003070, 0.000000);

        Mat rvec, tvec, rmat;
        solvePnP(boardPoints, pointBuf2d, cameraMatrix, distortionCoefficients, rvec, tvec, false, SOLVEPNP_ITERATIVE);

        std::vector<cv::Point2d> imagePoints;
        projectPoints(axis, rvec, tvec, cameraMatrix, distortionCoefficients, imagePoints);
        Rodrigues(rvec, rmat, noArray());

        Point2f corner = pointBuf[0];
        line(cv_ptr->image, corner, imagePoints[0], (255,0,0), 5);
        line(cv_ptr->image, corner, imagePoints[1], (0,255,0), 5);
        line(cv_ptr->image, corner, imagePoints[2], (0,0,255), 5);

//        cout << "Rotation Vector " << endl << rmat << endl;
//        cout << "Translation Vector " << endl << tvec << endl;

        imshow("image", cv_ptr->image);
        waitKey(1);
        pub.publish(cv_ptr->toImageMsg());

        static tf::TransformBroadcaster br;
        tf::Transform transform;
        transform.setOrigin(tf::Vector3(tvec.at<double>(0,0), tvec.at<double>(1,0), tvec.at<double>(2,0)));

        tf::Matrix3x3 tf3d;
        tf3d.setValue(rmat.at<double>(0,0), rmat.at<double>(0,1), rmat.at<double>(0,2),
              rmat.at<double>(1,0), rmat.at<double>(1,1), rmat.at<double>(1,2),
              rmat.at<double>(2,0), rmat.at<double>(2,1), rmat.at<double>(2,2));

        tf::Quaternion tfqt;
        tf3d.getRotation(tfqt);
        transform.setRotation(tfqt);
        br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "camera", "checker_board"));
        */
    }
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "image_node");
    ROS_INFO("Started Node");
    ros::NodeHandle nh;
    ros::Rate rate(20);
    ros::Subscriber imageSubscriber = nh.subscribe("/camera/color/image_raw", 1, imageCallBack);
    pub = nh.advertise < sensor_msgs::Image > ("/checker_matrix", 1);

    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}

