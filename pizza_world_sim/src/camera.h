// Header file for the camera implementation
// This Part of the code recieves the raw image from the three raspberry pi cameras
// Attatched to the robot. It then runs line following through the front camera
// and blob detection from the left and right cameras

#ifndef _CAMERA_H
#define _CAMERA_H

#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>
#include <iostream>
#include <cstdio>

static const cv::String FRONTCV_WINDOW = "FRONT CAMERA";
static const cv::String LEFTCV_WINDOW = "LEFT CAMERA";
static const cv::String RIGHTCV_WINDOW = "RIGHT CAMERA";

// Forward Declarations
class LineFollowing;
class IntersectionDet;
class MotionControl;
class BlobDetection;

class ImageConverter
{
public:
  ImageConverter()
  {}
  ~ImageConverter()
  {}
  cv::Mat Gauss(cv::Mat input);

  LineFollowing* _LineFollower;
  IntersectionDet* _IntersectionDetection;
  MotionControl* _MotionControl;
  BlobDetection* _BlobDetector;

private:
};

class LeftImageConverter: public ImageConverter
{
  public:
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    LeftImageConverter(LineFollowing* lf_, IntersectionDet * Id_, int RobotID, BlobDetection* Bd_)
      : it_(nh_)
    {
      _LineFollower = lf_;
      _IntersectionDetection = Id_;
      _BlobDetector = Bd_;

      char LeftCamSubString [50];
      sprintf(LeftCamSubString, "tb3_%d/left_camera/image",RobotID);
      image_sub_ = it_.subscribe(LeftCamSubString, 1, &LeftImageConverter::imageCb, this);
      //cv::namedWindow(LEFTCV_WINDOW);
    }

    ~LeftImageConverter()
    {
      //cv::destroyWindow(LEFTCV_WINDOW);
    }
    //getter function for robot brain
    bool BlobCheck();
  private:
    cv::Mat converted_image;
    cv_bridge::CvImagePtr cv_ptr;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    int detected_blob;
};


class RightImageConverter: public ImageConverter
{
  public:
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    RightImageConverter(LineFollowing* lf_, IntersectionDet * Id_, int RobotID, BlobDetection* Bd_)
      : it_(nh_)
    {
      _LineFollower = lf_;
      _IntersectionDetection = Id_;
      _BlobDetector = Bd_;

      char RightCamSubString [50];
      sprintf(RightCamSubString, "tb3_%d/right_camera/image",RobotID);
      image_sub_ = it_.subscribe(RightCamSubString, 1, &RightImageConverter::imageCb, this);

    //  cv::namedWindow(RIGHTCV_WINDOW);
    }
    ~RightImageConverter()
    {
    //  cv::destroyWindow(RIGHTCV_WINDOW);
    }
    //getter function for robot brain
    bool BlobCheck();
  private:
    cv::Mat converted_image;
    cv_bridge::CvImagePtr cv_ptr;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    bool detected_blob;
};

class FrontImageConverter: public ImageConverter
{
  public:
    void imageCb(const sensor_msgs::ImageConstPtr& msg);
    FrontImageConverter(LineFollowing* lf_, IntersectionDet * Id_, int RobotID, BlobDetection* Bd_)
      : it_(nh_)
    {
      _LineFollower = lf_;
      _IntersectionDetection = Id_;
      _BlobDetector = Bd_;

      char HeadCamSubString [50];
      sprintf(HeadCamSubString, "tb3_%d/head_camera/image",RobotID);
      image_sub_ = it_.subscribe(HeadCamSubString, 1, &FrontImageConverter::imageCb, this);

    // cv::namedWindow(FRONTCV_WINDOW);
    }

    ~FrontImageConverter()
    {
    //  cv::destroyWindow(FRONTCV_WINDOW);
    }

    double GetLineCentroid();
    double GetInterSectionCentroid();

  private:
    cv::Mat converted_image;
    cv_bridge::CvImagePtr cv_ptr;

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

    double c_I;
    double c_L;
};

#endif
