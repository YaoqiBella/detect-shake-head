#include <cmath>
#include "detector.h"

void Detector::addFrame(const cv::Mat& frame) {
  buffer_.push_back(frame);
  if (buffer_.size() > bufferSize_) {
    buffer_.pop_front();
  }
}


Direction Detector::classifyObjectPosition(cv::Mat& frame, const double threshold) {
  cv::Size s = frame.size();
  cv::Mat leftMat = frame(cv::Range::all(), cv::Range(s.width / 2, s.width));
  cv::Mat rightMat = frame(cv::Range::all(), cv::Range(0, s.width / 2));
  double leftSum = cv::sum(leftMat).val[0];
  double rightSum = cv::sum(rightMat).val[0];
  // Normalize the lightness.
  double totalSum = leftSum + rightSum;
  leftSum /= totalSum;
  rightSum /= totalSum;
  if (std::abs(leftSum - rightSum) < threshold) {
    return INVALID;
  }
  return leftSum > rightSum ? LEFT : RIGHT;
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
  seqAnalyzer_.addValue(classifyObjectPosition(sum, 0.2));
  return seqAnalyzer_.detectMovingDirection(1);
}
