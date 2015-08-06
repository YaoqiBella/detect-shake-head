#ifndef DETECTOR_H
#define DETECTOR_H
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

#include "common.h"

class Detector {
public:
  Detector(int bufferSize) {
    bufferSize_ = bufferSize;
  }

  Direction detectMotion(cv::Mat& sum);

  void addFrame(const cv::Mat& frame);
protected:
  std::list<cv::Mat> buffer_;
  int bufferSize_;
};

#endif // DETECTOR_H
