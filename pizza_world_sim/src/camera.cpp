/// camera.cpp
//
// Implementation file for the odometer, that can use the position provided by the
// callback to track the turtlebot’s lap number and if it is in the winning region of the maze
#include "camera.h"
#include "LineFollowing.h"
#include "IntersectionDetection.h"
#include "MotionControl.h"
#include "BlobDetection.h"

void FrontImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  std::cout<<"Front callback"<<std::endl;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  converted_image =cv_ptr->image;
   //cv::resizeWindow(FRONTCV_WINDOW,1200,800)#;
   cv::resize(cv_ptr->image,converted_image,cv::Size(1200,800),cv::INTER_LINEAR);

  //Implement the line following
  //imshow(FRONTCV_WINDOW, converted_image);
  //cv::waitKey(3);
  converted_image = Gauss(converted_image);

  c_I = _IntersectionDetection->colorthresh(converted_image);
  c_L = _LineFollower->colorthresh(converted_image);

  image_pub_.publish(cv_ptr->toImageMsg());
}

void RightImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  std::cout<<"Blob callback"<<std::endl;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  converted_image =cv_ptr->image;
   //cv::resizeWindow(RIGHTCV_WINDOW,1200,800);
   cv::resize(cv_ptr->image,converted_image,cv::Size(1200,800),cv::INTER_LINEAR);

  //Implement the line following
  //imshow(RIGHTCV_WINDOW, converted_image);
  //cv::waitKey(3);
  converted_image = Gauss(converted_image);

  detected_blob = _BlobDetector->BlobManager(converted_image);

  image_pub_.publish(cv_ptr->toImageMsg());
}


void LeftImageConverter::imageCb(const sensor_msgs::ImageConstPtr& msg)
{
  std::cout<<"Blob callback"<<std::endl;
  try
  {
    cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  converted_image =cv_ptr->image;

  // cv::resizeWindow(LEFTCV_WINDOW,1200,800);
   cv::resize(cv_ptr->image,converted_image,cv::Size(1200,800),cv::INTER_LINEAR);

  //Implement the line following
  //imshow(LEFTCV_WINDOW, converted_image);
  //cv::waitKey(3);
  converted_image = Gauss(converted_image);

  detected_blob = _BlobDetector->BlobManager(converted_image);

  image_pub_.publish(cv_ptr->toImageMsg());
}

cv::Mat ImageConverter::Gauss(cv::Mat input) {
  cv::Mat output;
// Applying Gaussian Filter
  //output = input;
  // Output images viewed by the turtlebot
  cv::GaussianBlur(input, output, cv::Size(3, 3), 0.1, 0.1);
  return output;
}

double FrontImageConverter::GetLineCentroid()
{
  return c_L;
}

double FrontImageConverter::GetInterSectionCentroid()
{
  return c_I;
}

bool LeftImageConverter::BlobCheck()
{
  bool result = false;

  if(detected_blob){
    result = true;
  }
  return result;
}

bool RightImageConverter::BlobCheck()
{
  bool result = false;

  if(detected_blob){
    result = true;
  }
  return result;
}
