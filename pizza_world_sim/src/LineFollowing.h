//LineFollowing.h

#ifndef _LINEFOLLOWING_H
#define _LINEFOLLOWING_H

#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <geometry_msgs/Twist.h>

static const cv::String LINEFOLLOW_WINDOW = "LINEFOLLOWING";


class LineFollowing
{
  public:
      LineFollowing(ros::NodeHandle nh);    // Constructor
      ~LineFollowing();                     // Destructor

      double colorthresh(cv::Mat input);


  private:

      ros::NodeHandle _MyNodeHandle;
      cv::Scalar LowerYellow;
      cv::Scalar UpperYellow;
      cv::Scalar LowerRed;
      cv::Scalar UpperRed;
      cv::Mat img_hsv;
      cv::Mat imgMaskRoad;
      cv::Mat imgMaskIntersection;
      double c_x;

      //approximately centre
      const int IdealCentroid = 568;

};

#endif
