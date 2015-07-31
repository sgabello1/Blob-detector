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
using namespace Eigen;

cv::Mat gray_img, bgr_img, rgb_img;
std::string pub_id;
cv_bridge::CvImagePtr cv_ptr;

vector<int> laser_point;
vector<vector<int> > abcd(4);    
int offset = 300;

cv::Rect laser_roi;
vector<float> vClick;
bool go = false;

void on_mouse( int e, int x, int y, int d, void *ptr )
{
    if (e == EVENT_LBUTTONDOWN )
    {

        laser_roi.x = float(x);
        laser_roi.y = float(y);
        laser_roi.width = 100;
        laser_roi.height = 100;
     
        go = true;
 
    }

   
}

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

    setMouseCallback("Display window",on_mouse, NULL );

    if(go)
      {
      if (!rgb_img.empty()){  
          cv::cvtColor(rgb_img, bgr_img, CV_RGB2BGR);
          cv::cvtColor(rgb_img, gray_img, CV_RGB2GRAY);
    
          if (look3d::detect_laser_4dots(gray_img, laser_roi, abcd))
                {
                cv::circle(bgr_img, cv::Point(abcd[0][0], abcd[0][1]), 15,
                           cv::Scalar(255, 0, 0)); // blue
                cv::circle(bgr_img, cv::Point(abcd[1][0], abcd[1][1]), 15,
                           cv::Scalar(0, 255, 0)); // green
                cv::circle(bgr_img, cv::Point(abcd[2][0], abcd[2][1]), 15,
                           cv::Scalar(0, 0, 255)); // red
                cv::circle(bgr_img, cv::Point(abcd[3][0], abcd[3][1]), 15,
                           cv::Scalar(255, 255, 0)); // blue + green
                }
              }
          else
          cout << "Detection failed " << endl;
    
      if (!gray_img.empty())
        cv::imshow("img", gray_img);
      if (!bgr_img.empty())
        cv::imshow("laser_detect", bgr_img);
      }

    
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "listener");
  
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/mv_29900485/image_raw/", 10, receivedImage);

  cout << "Please just click close to the upper left point.. " << endl;

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


/*

typedef Matrix< std::complex<double> , 8 , 2 > Matrix82;
    

int main(int argc, char **argv) {
   
    cv::Mat gray_img, bgr_img, rgb_img;
    
    vector<int> laser_point;
    vector<vector<int> > laser_locations;

    // laser_locations.resize(4);
    // laser_point.resize(2);

    int offset =  atoi(argv[1]); //suggested offset 200

    rgb_img = cv::imread(argv[2]); //eg /home/sgabello/Pictures/red_dot_pic1.jpg

    if (!rgb_img.empty()){  
      cv::cvtColor(rgb_img, bgr_img, CV_RGB2BGR);
      cv::cvtColor(rgb_img, gray_img, CV_RGB2GRAY);

      cv::Rect laser_roi(gray_img.cols / 2 - offset, gray_img.rows / 2 - offset,
                         2 * offset, 2 * offset);
      if (look3d::detect_laser_4dots(gray_img, laser_roi, laser_locations));
        cv::circle(bgr_img, cv::Point(laser_locations[0][0], laser_locations[0][1]), 15,
                   cv::Scalar(255, 0, 0));
        cv::circle(bgr_img, cv::Point(laser_locations[1][0], laser_locations[1][1]), 15,
                   cv::Scalar(255, 0, 0));
        cv::circle(bgr_img, cv::Point(laser_locations[2][0], laser_locations[2][1]), 15,
                   cv::Scalar(255, 0, 0));
        cv::circle(bgr_img, cv::Point(laser_locations[3][0], laser_locations[3][1]), 15,
                   cv::Scalar(255, 0, 0));
      // cout << "Detection ok  laser_location " << laser_locations <<  endl;
    }
    else
      cout << "Detection failed " << endl;

    if (!gray_img.empty())
      cv::imshow("img", rgb_img);
    if (!bgr_img.empty())
      cv::imshow("laser_detect", bgr_img);

    waitKey(0);
  } */
