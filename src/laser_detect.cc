#include <iostream>
#include <vector>

#include <Eigen/Eigen>
#include <boost/property_tree/ptree.hpp>
#include <boost/property_tree/ini_parser.hpp>

#include <opencv2/highgui/highgui.hpp>

#include "sense/sensor_replay.h"
#include "sense/videocapture_dispatch.h"
#include "sense/pd5x_laserdriver.h"

#include "laser_calib/feature_helpers.h"

static boost::property_tree::ptree config_pt;
static sense::SenseManager sense_manager;
std::string pub_id;

void create_publisher() {
  const auto fn = config_pt.get<std::string>("sensor.replay", "");
  if (fn.empty()) {

    std::shared_ptr<sense::VideoCaptureDispatch> videodispatch(
        new sense::VideoCaptureDispatch(0,
                                        sense::CameraFamily::PGR_FIREFLY_MV));
    videodispatch->SetProperty("Exposure", config_pt.get<int>("video.exposure"));
    videodispatch->SetProperty("Gain", config_pt.get<int>("video.gain"));
    videodispatch->SetProperty("Shutter", config_pt.get<int>("video.shutter"));
    sense_manager.Register(videodispatch);
    pub_id = videodispatch->identity();

    std::shared_ptr<sense::PD5xLaserdriver> laserdriver(new sense::PD5xLaserdriver(
        config_pt.get<std::string>("laser.serialport")));
    laserdriver->SetMeasuringMode(sense::PD5xLaserdriver::MULTIPLE);
    sense_manager.Register(laserdriver);
  } else {
    const bool loop = config_pt.get<bool>("sensor.loop", true);
    std::shared_ptr<sense::SensorReplay> replay(new sense::SensorReplay(fn, loop));
    sense_manager.Register(replay);
    pub_id = replay->identity();
  }
}

int main(int argc, char **argv) {
  std::string configfile =
      (argc > 1) ? argv[1] : std::string(Look3D_ROOT) +
                                 "/apps/laser_calib/calibration.ini";
  printf(" loading config file from: %s\n", configfile.c_str());
  boost::property_tree::ini_parser::read_ini(configfile, config_pt);

  create_publisher();
  sense::SenseSubscriberPtr video_rec(new sense::SenseSubscriber());
  if (!sense_manager.Register(video_rec, pub_id)) {
    printf("Cannot register video receiver to publisher(%s)\n", pub_id.c_str());
    return -1;
  }

  bool stop = false, pausing = false;
  cv::Mat gray_img, bgr_img, rgb_img;
  Eigen::Vector2d laser_location;
  while (!stop) {
    uint8_t key = cv::waitKey(20);
    switch (key) {
    case 'n':
      break;
    case 27:
      stop = true;
      break;
    case 'p':
      pausing = !pausing;
      break;
    }
    sense::ImageEventPtr evt;
    if (!video_rec->empty())
      evt  = std::dynamic_pointer_cast<sense::ImageEvent>(video_rec->pop());
    if (evt)
      rgb_img = evt->frame();
    if (!pausing && !rgb_img.empty()) {
      cv::cvtColor(rgb_img, bgr_img, CV_RGB2BGR);
      cv::cvtColor(rgb_img, gray_img, CV_RGB2GRAY);

      int offset = 100;
      cv::Rect laser_roi(gray_img.cols / 2 - offset, gray_img.rows / 2 - offset,
                         2 * offset, 2 * offset);
      if (look3d::detect_laser_dot(gray_img, laser_roi, laser_location))
        cv::circle(bgr_img, cv::Point(laser_location[0], laser_location[1]), 3,
                   cv::Scalar(255, 0, 0));
    }

    if (!gray_img.empty())
      cv::imshow("img", gray_img);
    if (!bgr_img.empty())
      cv::imshow("laser_detect", bgr_img);
  }

  sense_manager.clear();
}