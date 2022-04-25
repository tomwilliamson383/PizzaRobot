//intersectionDet.h

#ifndef _INTERSECTIONDET_H
#define _INTERSECTIONDET_H


#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <geometry_msgs/Twist.h>

static const cv::String INTERSECTION_WINDOW = "INTERSECT DETECTION";


class IntersectionDet
{
  public:
      IntersectionDet(ros::NodeHandle nh);    // Constructor
      ~IntersectionDet();                     // Destructor

      double colorthresh(cv::Mat input);


  private:
      ros::Publisher pub_vel_;
      ros::NodeHandle _MyNodeHandle;
      cv::Scalar LowerRed;
      cv::Scalar UpperRed;
      cv::Mat img_hsv;
      cv::Mat imgMaskRoad;

      double c_x;

};

#endif
