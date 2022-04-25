// Header file for Blob detection functionality in the Robot brain
// This will be instantiated in the functionality when the robot is currently
// moving on its way to the final destination Node
// It will track and then find the node location using blob detection, a
// combination of thresholding and contour analysis using open cv
// It takes in the cv formatted image matrix and outputs the state of Detection

#ifndef _BLOBDETECTION_H
#define _BLOBDETECTION_H

#include <ros/ros.h>
#include <ros/console.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui.hpp>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <geometry_msgs/Twist.h>

static const cv::String LEFT_BLOB_WINDOW = "LEFT WINDOW";
static const cv::String RIGHT_BLOB_WINDOW = "RIGHT WINDOW";
static const cv::String FRONT_BLOB_WINDOW = "FRONT WINDOW";

class BlobDetection
{
public:
  BlobDetection(ros::NodeHandle nh, int camera_ID);
  ~BlobDetection();

  bool BlobManager(cv::Mat input);
  void detectCustomer(cv::Mat input);
  void detectRestaurant(cv::Mat input);

private:
  int blob_ID; // Left right or front
  bool blobDetected; // Boolean triggered when blob is found within range

  int MIN_THRESHHOLD; // square size min limit for blob rectangle
  int MAX_THRESHOLD; // max size for rectangle

  ros::NodeHandle _MyNodeHandle;

  // Colour limits for Customer and Restaurant
  cv::Scalar LowerYellow;
  cv::Scalar UpperYellow;
  cv::Scalar LowerBlue;
  cv::Scalar UpperBlue;

  // Vector of contours
  std::vector<std::vector<cv::Point> > contour_vector;
  std::vector<cv::Vec4i> hierarchy;

  // Image matrix to be copied into the
  cv::Mat img_hsv_customer;
  cv::Mat img_hsv_restaurant;
  cv::Mat img_hsv_charger;

  cv::Mat imgMaskRestaurant;
  cv::Mat imgMaskCustomer;
  cv::Mat imgMaskCharger;

  cv::String BLOB_WINDOW = "UNNAMED";

};
#endif
