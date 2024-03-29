#include <iostream>
#include <fstream>
#include <Eigen/Core>

// opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <look3d/track/plane_tracker.h>

#ifdef ANDROID

#ifndef LOG
#define LOG(...)  //__android_log_print(ANDROID_LOG_INFO, LOG_TAG, __VA_ARGS__)
#endif

#ifndef LOGE
#define LOGE(...)  //__android_log_print(ANDROID_LOG_ERROR, LOG_TAG, __VA_ARGS__)
#endif

#else 

#define LOG(...)
#define LOGE(...)

#endif

// Gets current projection matrix (= PerspectiveMatrix * CameraPoseMatrix)
Eigen::Matrix<double, 3, 4> get_projection(look3d::PlaneTracker& tracker) {
    std::vector<double> cam_params = tracker.GetCameraParams();
    Eigen::Matrix3d intrinsics = Eigen::Matrix3d::Identity();
    intrinsics(0, 0) = cam_params[0];
    intrinsics(1, 1) = cam_params[1];
    intrinsics(0, 2) = cam_params[2];
    intrinsics(1, 2) = cam_params[3];

    std::vector<double> m = tracker.GetCurrentPose();
    Eigen::Matrix<double, 4, 4> mtr;
    mtr << m[0], m[1], m[2], m[3],
            m[4], m[5], m[6], m[7],
            m[8], m[9], m[10], m[11],
            m[12], m[13], m[14], m[15];
    Eigen::Matrix<double, 3, 4> pose;
    pose.setZero();
    pose.block<3, 3>(0, 0) = mtr.block<3, 3>(0, 0);
    pose.block<3, 1>(0, 3) = mtr.block<3, 1>(0, 3);

    Eigen::Matrix<double, 3, 4> projection = intrinsics * pose;
    return projection;
}

// Draws desirable target in world coordinate to current color image
void draw_target(cv::Mat& rgb_img, look3d::PlaneTracker& tracker) {
    const Eigen::Vector4d point_target(0, 0, 0, 1);
    Eigen::Matrix<double, 3, 4> proj = get_projection(tracker);
    Eigen::Vector3d point_cam = proj * point_target;
    point_cam /= point_cam[2];
    Eigen::Vector3d pointx_cam = proj * (point_target + Eigen::Vector4d(0.1, 0, 0, 1));
    pointx_cam /= pointx_cam[2];
    Eigen::Vector3d pointy_cam = proj * (point_target + Eigen::Vector4d(0, 0.1, 0, 1));
    pointy_cam /= pointy_cam[2];
    Eigen::Vector3d pointz_cam = proj * (point_target + Eigen::Vector4d(0, 0, 0.1, 1));
    pointz_cam /= pointz_cam[2];
    cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]),
             cv::Point(pointx_cam[0], pointx_cam[1]), cv::Scalar(0, 0, 255), 3);
    cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]),
             cv::Point(pointy_cam[0], pointy_cam[1]), cv::Scalar(0, 255, 0), 3);
    cv::line(rgb_img, cv::Point(point_cam[0], point_cam[1]),
             cv::Point(pointz_cam[0], pointz_cam[1]), cv::Scalar(255, 0, 0), 3);
}


using namespace look3d;
int main(int argc, char ** argv){
  /// initialize camera capture
  cv::VideoCapture capture;
#ifdef ANDROID
  if (!capture.open(CV_CAP_ANDROID + 0)) {
    LOGE("ERROR: Cannot open cv::VideoCapture");
    return -1;
  }
#else
  if (!capture.open(0)) {
    printf("ERROR: Cannot open cv::VideoCapture\n");
    return -1;
  }
#endif
  capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
  capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
  
  /// initialize the tracker
  PlaneTracker tracker;
  // std::string config_file = (argc > 1) ? argv[1] : 
  //   std::string(Look3D_ROOT) + "/look3d/examples/plane_tracker/plane_tracker.ini" ;
  const std::string config_file =
      std::string("/home/sgabello/pose_estimation_ws/src/marker_tracker/src/marker_tracker.ini");



  if (!tracker.Configure(config_file)) {
    printf("Configuration fails (file:%s)\n", config_file.c_str());
    return -1;
  }
  tracker.RestartTracker();

  bool stop = false;
  while (!stop) {
    static cv::Mat color_img;
    capture.read(color_img);
    if (!color_img.empty()) {
      static cv::Mat gray_img;
      cv::cvtColor(color_img, gray_img, CV_BGR2GRAY);
      PlaneTracker::TrackResult res = tracker.Track(gray_img);
      std::cout << "Tracking result:" << tracker.TrackResultToStr(res) << std::endl;

      cv::Mat cloned_img = color_img.clone();
      draw_target(cloned_img, tracker);
      cv::imshow("Plane Tracker", cloned_img);
    }
    char c = cv::waitKey(5);
    switch (c) {
      case 27:
      stop = true;
      break;
      case 'r':
      tracker.RestartTracker();
      break;
    }
  }

  return 0;
}
