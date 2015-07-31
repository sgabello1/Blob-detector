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
int offset = 300;

cv::Rect laser_roi;
vector<float> vClick;
bool go = false;
int n_point = 0;
vector<int> laser_location(2);
vector<vector<int> > abcd(4);    

void drawSelectedPoints(){

  for(int i = 0 ; i < n_point; i++)
  {

    cout << " n_point " << n_point <<" abcd[i][0] " << abcd[i][0] << " abcd[i][1] "<< abcd[i][1] << " i " << i << endl;
    cv::circle(bgr_img, cv::Point(abcd[i][0],abcd[i][1]), 15,
               cv::Scalar(0, 0, 255)); // red
    if (!bgr_img.empty())
    cv::imshow("laser_detect", bgr_img);

  }
}

void on_mouse( int e, int x, int y, int d, void *ptr )
{
    if (e == EVENT_LBUTTONDOWN && n_point < 4)
    {

        laser_roi.x = float(x);
        laser_roi.y = float(y);
        laser_roi.width = 15;
        laser_roi.height = 15;
     
     if (!rgb_img.empty()){  
      cv::cvtColor(rgb_img, bgr_img, CV_RGB2BGR);
      cv::cvtColor(rgb_img, gray_img, CV_RGB2GRAY);

      if (look3d::detect_laser_dot_click(gray_img, laser_roi, laser_location))
            {

            abcd[n_point] = laser_location;
            int s = abcd.size();
            // cout << " Point " << n_point << " at (" << laser_location[0] << " , " << laser_location[1] << ")"<< endl; 
            // cout << " size of abcd " << s << endl;
            cout << " Point " << n_point << " at (" << abcd[n_point][0] << " , " << abcd[n_point][1] << ")"<< endl; 
            n_point++;
            drawSelectedPoints();

            }
          }
          else
          {
            cout << "Detection failed " << endl;
            n_point = 0;
          }

  // if (!gray_img.empty())
  //   cv::imshow("img", gray_img);
  

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
