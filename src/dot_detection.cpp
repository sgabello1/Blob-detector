#include <iostream>
#include <vector>

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

std::string pub_id;

using namespace cv;
using namespace std;

int main(int argc, char **argv) {
   
    cv::Mat gray_img, bgr_img, rgb_img;

    Eigen::Vector2d laser_location;
    
    int offset =  atoi(argv[1]); //suggested offset 200

    rgb_img = cv::imread(argv[2]); //eg /home/sgabello/Pictures/red_dot_pic1.jpg

    if (!rgb_img.empty()){  
      cv::cvtColor(rgb_img, bgr_img, CV_RGB2BGR);
      cv::cvtColor(rgb_img, gray_img, CV_RGB2GRAY);

      cv::Rect laser_roi(gray_img.cols / 2 - offset, gray_img.rows / 2 - offset,
                         2 * offset, 2 * offset);
      if (look3d::detect_laser_dot(gray_img, laser_roi, laser_location))
        cv::circle(bgr_img, cv::Point(laser_location[0], laser_location[1]), 15,
                   cv::Scalar(255, 0, 0));
      cout << "Detection ok  laser_location " << laser_location[0] << " , "<<laser_location[1] <<  endl;
    }
    else
      cout << "Detection failed " << endl;

    if (!gray_img.empty())
      cv::imshow("img", rgb_img);
    if (!bgr_img.empty())
      cv::imshow("laser_detect", bgr_img);

    waitKey(0);
  }
