#include <iostream>
#include <vector>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/Image.h"
#include <image_transport/image_transport.h>

//  problem with compressed image

#include <Eigen/Eigen>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include "opencv2/core/core.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/nonfree/nonfree.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "feature_helpers.h"

using namespace cv;
using namespace std;

cv::Mat gray_img, bgr_img, rgb_img;
std::string pub_id;
cv_bridge::CvImagePtr cv_ptr;

Eigen::Vector2d laser_location;
    
int offset = 200;


void receivedImage(const sensor_msgs::Image::ConstPtr& img)
{

    try
    {
      cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
      rgb_img=cv_ptr->image; //  scene viewed by the drone (background + desktop)

    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    namedWindow( "Display window" );// Create a window
    imshow( "Display window",rgb_img);

      if (!rgb_img.empty()){  
    cv::cvtColor(rgb_img, bgr_img, CV_RGB2BGR);
    cv::cvtColor(rgb_img, gray_img, CV_RGB2GRAY);

    cv::Rect laser_roi(gray_img.cols / 2 - offset, gray_img.rows / 2 - offset,
                       2 * offset, 2 * offset);
    if (look3d::detect_laser_dot(gray_img, laser_roi, laser_location))
      cv::circle(bgr_img, cv::Point(laser_location[0], laser_location[1]), 3,
                 cv::Scalar(255, 0, 0));
    cout << "Detection ok  laser_location " << laser_location[0] << " , "<<laser_location[1] <<  endl;
  }
  else
    cout << "Detection failed " << endl;

  if (!gray_img.empty())
    cv::imshow("img", gray_img);
  if (!bgr_img.empty())
    cv::imshow("laser_detect", bgr_img);

    
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "listener");
  
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/mv_29900485/image_raw/compressed", 10, receivedImage);


  //  Press "q button" to exit
  while(1)
   {
      int key=waitKey(10);
      
      // cout << key  << endl;

      if((char)key=='q') break;
      ros::spinOnce();
   }
  return 0;
   
  }
