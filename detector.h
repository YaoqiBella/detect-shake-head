#ifndef DETECTOR_H
#define DETECTOR_H
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

#include "common.h"

class SequenceAnalyzer {
public:
  SequenceAnalyzer(int bufferSize) {
    bufferSize_ = bufferSize;
  }

  void addValue(const int& value) {
    if (value == -1) {
      return;
    }

    buffer_.push_back(value);
    if (buffer_.size() > bufferSize_) {
      buffer_.pop_front();
    }
  }

  // void majorityVote(const std::list<Direction>& window) {
  // }

  Direction detectMovingDirection(int threshold) {
    std::list<int>::iterator li = buffer_.begin();
    int pre = *li, cur;
    ++li;
    int leftVote = 0;
    int rightVote = 0;
    while (li != buffer_.end()) {
      cur = *li;
      if (pre < cur) {
        ++rightVote;
      } else if (pre > cur) {
        ++leftVote;
      }
      pre = cur;
      ++li;
    }
    std::cout << "leftVote: " << leftVote << std::endl;
    std::cout << "rightVote: " << rightVote << std::endl;
    // if (leftVote <= threshold && rightVote <= threshold) {
    //   return INVALID;
    // }
    if (leftVote == rightVote) {
      return INVALID;
    }
    return leftVote > rightVote ? LEFT : RIGHT;
  }

protected:
  std::list<int> buffer_;
  int bufferSize_;
};

class Detector {
public:
  Detector(int bufferSize) : seqAnalyzer_(3) {
    bufferSize_ = bufferSize;
  }

  int classifyObjectPosition(cv::Mat& frame, const double threshold);
  Direction detectMotion(cv::Mat& sum);

  void addFrame(const cv::Mat& frame);
protected:
  std::list<cv::Mat> buffer_;
  int bufferSize_;
  SequenceAnalyzer seqAnalyzer_;
};

#endif // DETECTOR_H
