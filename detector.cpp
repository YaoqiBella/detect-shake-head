#include <cmath>
#include <vector>
#include "detector.h"

void MotionDetector::addFrame(const cv::Mat& frame) {
  buffer_.push_back(frame);
  if (buffer_.size() > bufferSize_) {
    buffer_.pop_front();
  }
}


bool MotionDetector::extractForeground(cv::Mat& sum) {
  if (buffer_.size() != bufferSize_) {
    std::cout << "unknown buffer_ size! " << std::endl;
    return false;
  }
  cv::Mat d, pre;
  std::list<cv::Mat>::iterator li = buffer_.begin();
  pre = *li;
  ++li;
  while (li != buffer_.end()) {
    cv::absdiff(pre, *li, d);
    cv::add(sum, d, sum);
    pre = *li;
    ++li;
  }
  return true;
}

Position MotionDetector::identifyObjectPosition(cv::Mat& frame, const int gridWidth, const double threshold) {
  cv::Size s = frame.size();
  std::vector<double> lightness(6, 0);
  int regionWidth = s.width / gridWidth_;

  double maxLight = -1, sumLight = 0;
  int maxLightRegion = -1;
  for (int i = 0; i < gridWidth; ++i) {
    cv::Mat regionMat = frame(cv::Range::all(), 
                              cv::Range((gridWidth - i - 1) * regionWidth, 
                                        (gridWidth - i) * regionWidth));
    lightness[i] = cv::sum(regionMat).val[0];
    if (lightness[i] > maxLight) {
      maxLight = lightness[i];
      maxLightRegion = i;
    }
    sumLight += lightness[i];
  }
  // std::cout << "maxLight: " << maxLight << std::endl;
  // std::cout << "sumLight: " << sumLight << std::endl;
  std::cout << "maxLight / sumLight: " << maxLight / sumLight;
  std::cout << "maxLightRegion: " << maxLightRegion<< std::endl;
  if (maxLight / sumLight  <  threshold / gridWidth) {
    return -1;
  }
  return maxLightRegion;
}

Position MotionDetector::detect(cv::Mat& foreground) {
  extractForeground(foreground);
  return identifyObjectPosition(foreground, gridWidth_, motionThreshold_);
  // std::cout << "position: " << position << std::endl;
  // seqAnalyzer_.addValue(position);
  // return seqAnalyzer_.detectMovingDirection(1);
}
