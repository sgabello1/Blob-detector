#include <iostream>
#include <vector>

#include <Eigen/Eigen>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>

// #include "sense/tracking_events.h"
// #include "sense/videocapture_dispatch.h"

// #include "laser_calib/feature_helpers.h"

static boost::property_tree::ptree config_pt;
static cv::Ptr<cv::FeatureDetector> ptr_featuredetector;

void init_detector() {
  cv::SimpleBlobDetector::Params params;
  params.thresholdStep = 10;
  params.minThreshold = 50;
  params.maxThreshold = 220;
  params.minRepeatability = 2;
  params.minDistBetweenBlobs = 3; // adjusted; original:10

  params.filterByColor = true;
  params.blobColor = 0;

  params.filterByArea = true;
  params.minArea = 10; // adjusted; orignal: 25
  params.maxArea = 5000;

  params.filterByCircularity = false;
  params.minCircularity = 0.8f;
  params.maxCircularity = 1.e+9; // std::numeric_limits<float>::max();

  params.filterByInertia = true;
  // minInertiaRatio = 0.6;
  params.minInertiaRatio = 0.1f;
  params.maxInertiaRatio = 1.e+9; // std::numeric_limits<float>::max();

  params.filterByConvexity = true;
  // minConvexity = 0.8;
  params.minConvexity = 0.5f;  // adjusted, original 0.95
  params.maxConvexity = 1.e+9; // std::numeric_limits<float>::max();

  ptr_featuredetector =
      cv::Ptr<cv::FeatureDetector>(new cv::SimpleBlobDetector(params));
}

int main(int argc, char **argv) {
  // std::string configfile =
  //     (argc > 1) ? argv[1] : std::string(Look3D_ROOT) +
  //                                "/apps/laser_calib/calibration.ini";
  // std::cout << " loading config file from: " << configfile << std::endl;

  // boost::property_tree::ini_parser::read_ini(configfile, config_pt);

  // sense::VideoCaptureDispatch videodispatch(0, sense::CameraFamily::PGR_FIREFLY_MV);
  // videodispatch.SetProperty("Exposure", config_pt.get<int>("video.exposure"));
  // videodispatch.SetProperty("Gain", config_pt.get<int>("video.gain"));
  // videodispatch.SetProperty("Shutter", config_pt.get<int>("video.shutter"));

  // sense::SenseSubscriber video_rec;
  // video_rec.Register(videodispatch);

  bool stop = false;
  char key = '0';
  cv::Mat gray_img, bgr_img, rgb_img, binary;
  std::vector<cv::Point2f> corners;
  cv::Size board_size = cv::Size(640,480);
      // cv::Size(config_pt.get<int>("calibration.horizontal_corners", 8),
      //          config_pt.get<int>("calibration.vertical_corners", 6));
  init_detector();
  // while (!stop) {
  //   while (!video_rec.empty())
  //     rgb_img = std::dynamic_pointer_cast<sense::ImageEvent>(video_rec.pop())
  //                   ->frame();

  rgb_img = cv::img_load("/home/sgabello/catkin_ws/rgb1.png");

    // if (key != 'p') {
      cv::cvtColor(rgb_img, bgr_img, CV_RGB2BGR);
      cv::cvtColor(rgb_img, gray_img, CV_RGB2GRAY);
      if (cv::findCirclesGrid(gray_img, board_size, corners,
                              cv::CALIB_CB_SYMMETRIC_GRID,
                              ptr_featuredetector)) {
        cv::Mat m_corners(1, corners.size(), CV_32FC2, &corners[0]);
        cv::drawChessboardCorners(bgr_img, board_size, m_corners, true);
      }
    // }

    cv::imshow("circlepattern_detect", bgr_img);
    char key = cv::waitKey(0);
    // switch (key) {
    // case 'n':
    //   break;
    // case 27:
    //   stop = true;
    //   break;
    // }
  // }
  return 0;
}