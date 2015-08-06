#include "detector.h"

void Detector::addFrame(const cv::Mat& frame) {
  buffer_.push_back(frame);
  if (buffer_.size() > bufferSize_) {
    buffer_.pop_front();
  }
}


Direction Detector::detectMotion(cv::Mat& sum) {
  if (buffer_.size() != bufferSize_) {
    std::cout << "unknown buffer_ size! " << std::endl;
    return INVALID;
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
  return INVALID;
}
