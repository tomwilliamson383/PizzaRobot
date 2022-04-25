#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "IntersectionDetection.h"

IntersectionDet::IntersectionDet( ros::NodeHandle nh) // Constructor
: _MyNodeHandle(nh)
{
ros::Rate loop_rate( 10 );

}

IntersectionDet::~IntersectionDet() // Destructor
{
cv::namedWindow(INTERSECTION_WINDOW);

}


//This function uses open source code that interacts with openCV to detect
//a rectangle in the imageconverter. The code draws inspiration from the following
// code source with full permision from the authors.

//arjunskumar, Line-Follower--RO (August 2021), Github Repository (and Youtube Video)
//    https://github.com/arjunskumar/Line-Follower--ROS

//The additional sources were also used and inspired from:

//Programmer Click, ROS line programming, (2021)
//    https://programmerclick.com/article/9343957456/

//OpenCV, Learn Open CV, Image Threshholding in OpenCV (2021),
//    https://learnopencv.com/opencv-threshold-python-cpp/

//This function finds a red box and produces a rectangle in a vision WINDOW
//the centroid of the rectangle is returned and then used in the PD controller
double IntersectionDet::colorthresh(cv::Mat input) {
  // Initializaing variables
  cv::Size s = input.size();
  std::vector<std::vector<cv::Point> > v;
  auto w = s.width;
  auto h = s.height;

  // Detect all objects within the HSV range
  cv::cvtColor(input, IntersectionDet::img_hsv, CV_BGR2HSV);


  //the threshold values for Red in BGR format
  IntersectionDet::LowerRed = {0, 165, 165};
  IntersectionDet::UpperRed = {0, 255, 255};


  cv::inRange(IntersectionDet::img_hsv, LowerRed,
     UpperRed, IntersectionDet::imgMaskRoad);

  imgMaskRoad(cv::Rect(0, 0, w, 0.8*h)) = 0;
  // Find contours for better visualization
  cv::findContours(IntersectionDet::imgMaskRoad, v, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
  // If contours exist add a bounding
  // Choosing contours with maximum area
  if (v.size() != 0) {
    auto area = 0;
    auto idx = 0;
    auto count = 0;
    while (count < v.size()) {
      if (area < v[count].size()) {
         idx = count;
         area = v[count].size();
      }
      count++;
    }
    cv::Rect rect = boundingRect(v[idx]);
    cv::Point pt1, pt2, pt3;
    pt1.x = rect.x;
    pt1.y = rect.y;
    pt2.x = rect.x + rect.width;
    pt2.y = rect.y + rect.height;
    pt3.x = pt1.x+5;
    pt3.y = pt1.y-5;
    // Drawing the rectangle using points obtained
    rectangle(input, pt1, pt2, CV_RGB(0, 0, 255), 2);
    // Inserting text box

    cv::putText(input, "Intersection Detection", pt3,
      CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(0, 0, 255));


  }
  // Mask image to limit the future turns affecting the output
  imgMaskRoad(cv::Rect(0.7*w, 0, 0.3*w, h)) = 0;
  imgMaskRoad(cv::Rect(0, 0, 0.3*w, h)) = 0;
  // Perform centroid detection of line
  cv::Moments M = cv::moments(IntersectionDet::imgMaskRoad);
  if (M.m00 > 0) {
    cv::Point p1(M.m10/M.m00, M.m01/M.m00);
    cv::circle(IntersectionDet::imgMaskRoad, p1, 5, cv::Scalar(155, 200, 0), -1);
  }

  if(M.m00 != 0 && M.m00 != 0){
    c_x = M.m10/M.m00;
  }
  else{
    c_x = 0.0;
  }
  // Tolerance to chooise directions

  auto count = cv::countNonZero(imgMaskRoad);


  // Output images viewed by the turtlebot
  //imshow(INTERSECTION_WINDOW, input);
  cv::waitKey(3);

  return c_x;
}
