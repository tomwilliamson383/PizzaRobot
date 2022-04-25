#include <vector>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "LineFollowing.h"

LineFollowing::LineFollowing( ros::NodeHandle nh) // Constructor
: _MyNodeHandle(nh)
{
  ros::Rate loop_rate( 10 );
  LineFollowing::LowerYellow = {20, 100, 100};
  LineFollowing::UpperYellow = {30, 255, 255};

  //set as the centre to start off
  c_x = 568;
  cv::namedWindow(LINEFOLLOW_WINDOW);

}

LineFollowing::~LineFollowing() // Destructor
{
  cv::destroyWindow(LINEFOLLOW_WINDOW);
}


// This function uses open source code that interacts with openCV to detect
// a rectangle in the imageconverter. The code draws inspiration from the following
// code source with full permision from the authors.

//arjunskumar, Line-Follower--RO (August 2021), Github Repository (and Youtube Video)
//    https://github.com/arjunskumar/Line-Follower--ROS

//The additional sources were also used and inspired from:

//Programmer Click, ROS line programming, (2021)
//    https://programmerclick.com/article/9343957456/

//OpenCV, Learn Open CV, Image Threshholding in OpenCV (2021),
//    https://learnopencv.com/opencv-threshold-python-cpp/

//This function finds a yellow line and produces a rectangle in a vision WINDOW
//the centroid of the rectangle is returned and then used in the PD controller

double LineFollowing::colorthresh(cv::Mat input) {
  // Initializaing variables
  cv::Size s = input.size();
  std::vector<std::vector<cv::Point> > v;
  auto w = s.width;
  auto h = s.height;

  // Detect all objects within the HSV range
  cv::cvtColor(input, LineFollowing::img_hsv, CV_BGR2HSV);


  cv::inRange(LineFollowing::img_hsv, LowerYellow,
     UpperYellow, LineFollowing::imgMaskRoad);

  imgMaskRoad(cv::Rect(0, 0, w, 0.8*h)) = 0;
  // Find contours for better visualization
  cv::findContours(LineFollowing::imgMaskRoad, v, CV_RETR_LIST, CV_CHAIN_APPROX_NONE);
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
    rectangle(input, pt1, pt2, CV_RGB(255, 0, 0), 2);
    // Inserting text box

    cv::putText(input, "Line Detected", pt3,
      CV_FONT_HERSHEY_COMPLEX, 1, CV_RGB(255, 0, 0));


  }
  // Mask image to limit the future turns affecting the output
  imgMaskRoad(cv::Rect(0.7*w, 0, 0.3*w, h)) = 0;
  imgMaskRoad(cv::Rect(0, 0, 0.3*w, h)) = 0;
  // Perform centroid detection of line
  cv::Moments M = cv::moments(LineFollowing::imgMaskRoad);
  if (M.m00 > 0) {
    cv::Point p1(M.m10/M.m00, M.m01/M.m00);
    cv::circle(LineFollowing::imgMaskRoad, p1, 5, cv::Scalar(155, 200, 0), -1);
  }

  //avoids NaN
  if(M.m00 != 0 && M.m00 != 0){
  c_x = M.m10/M.m00;
  }


  auto count = cv::countNonZero(imgMaskRoad);


  // Output images viewed by the turtlebot
  imshow(LINEFOLLOW_WINDOW, input);
  cv::waitKey(3);

  return c_x;
}
