#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "BlobDetection.h"

BlobDetection::BlobDetection( ros::NodeHandle nh, int camera_ID ) // Constructor
: _MyNodeHandle(nh), blob_ID(camera_ID), MIN_THRESHHOLD(200), MAX_THRESHOLD(320),blobDetected(false)
{
  ros::Rate loop_rate( 10 );
  BlobDetection::LowerYellow = {20, 100, 100};
  BlobDetection::UpperYellow = {30, 255, 255};
  BlobDetection::LowerBlue = {50,100,200};
  BlobDetection::UpperBlue = {100,255,255};

}

BlobDetection::~BlobDetection()
{
  //cv::destroyWindow(BLOB_WINDOW);
}
bool BlobDetection::BlobManager(cv::Mat input)
{
  //std::cout<<"BlobManaga"<<std::endl;
  switch (blob_ID)
  {
    case 0:
      BLOB_WINDOW = FRONT_BLOB_WINDOW;
      break;
    case 1:
      BLOB_WINDOW = LEFT_BLOB_WINDOW;
      break;
    case 2:
      BLOB_WINDOW = RIGHT_BLOB_WINDOW;
      break;
    default:
    break;
  }
  detectCustomer(input);
  detectRestaurant(input);
  //cv::namedWindow(BLOB_WINDOW);
  //imshow(BLOB_WINDOW,input);
  // cv::waitKey(5);
  return blobDetected;
}

// Will threshold input image from camera to observe objects in the yellow HSV
// colour scheme. It will then draw contours around these shapes. If a triangle
// is found (3 sided contour) and the dimensions are within range of it being
// next to the turtlebot, the customer will have been detected.
void BlobDetection::detectCustomer(cv::Mat input) {
  blobDetected = false;
  // Convert to HSV Coding
  cv::cvtColor(input, img_hsv_customer, CV_BGR2HSV);
  // Colour threshold to isolate yellow colours
  cv::inRange(img_hsv_customer, LowerYellow, UpperYellow, imgMaskCustomer);
  // Find the contours in the image
  cv::findContours( imgMaskCustomer, contour_vector, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );

  /// Approximate contours to polygons + get bounding rects and circles
  std::vector<std::vector<cv::Point> > contours_poly( contour_vector.size() );
  std::vector<cv::Rect> boundRect( contour_vector.size() );

  for( int i = 0; i < contour_vector.size(); i++ )
   {
     cv::approxPolyDP( cv::Mat(contour_vector[i]), contours_poly[i], 3, true );
     if (contours_poly[i].size() >=3 && contours_poly[i].size()<=7){
       boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );

       if (boundRect[i].width > MIN_THRESHHOLD && boundRect[i].height > MIN_THRESHHOLD && boundRect[i].width < MAX_THRESHOLD)
       {
          //std::cout<<"CUSTOMER!!"<<std::endl;
          blobDetected = true;
          cv::rectangle(input, cv::boundingRect(contours_poly[i]), cv::Scalar(0, 100, 255), 5);
       }
     }
     //cv::drawContours( input, contours_poly, i, cv::Scalar(0, 100, 255), 1, 8, std::vector<cv::Vec4i>(), 0, cv::Point() );
   }
}

// Will threshold input image from camera to observe objects in the blue HSV
// colour scheme. It will then draw contours around these shapes. If a rectangle
// is found (4 sided contour) and the dimensions are within range of it being
// next to the turtlebot, the customer will have been detected.
void BlobDetection::detectRestaurant(cv::Mat input) {
  blobDetected = false;
  // Converts BGR coded cv Matrix to HSV colour scheme
  cv::cvtColor(input, img_hsv_restaurant, CV_BGR2HSV);
  // Thresholds image to filter by blue images only
  cv::inRange(img_hsv_restaurant, LowerBlue,
    UpperBlue, imgMaskRestaurant);

  // Approximate contours to polygons + get bounding rects
  std::vector<std::vector<cv::Point> > contours_poly( contour_vector.size() );
  std::vector<cv::Rect> boundRect( contour_vector.size() );

  // Goes through each contour found in the thresholded image
  for( int i = 0; i < contour_vector.size(); i++ )
     {
       // Approximates a polygon from the contours found
       cv::approxPolyDP( cv::Mat(contour_vector[i]), contours_poly[i], 3, true );
       // If the polygon has at least 4 lines (sides) it is a square
       // Has a range due to
       if (contours_poly[i].size() >=4 && contours_poly[i].size()<=8){

        boundRect[i] = cv::boundingRect( cv::Mat(contours_poly[i]) );

        if (boundRect[i].width > MIN_THRESHHOLD && boundRect[i].height > MIN_THRESHHOLD && boundRect[i].width < MAX_THRESHOLD)
        {
          //std::cout<<"RESTAURANT!!"<<std::endl;
          blobDetected = true;
          cv::rectangle(input, cv::boundingRect(contours_poly[i]), cv::Scalar(255, 0, 0), 5);
        }
      }
     }
  }
