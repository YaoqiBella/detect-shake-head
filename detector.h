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

  void addValue(const Direction& value) {
    buffer_.push_back(value);
    if (buffer_.size() > bufferSize_) {
      buffer_.pop_front();
    }
  }

  // void majorityVote(const std::list<Direction>& window) {
  // }

  Direction detectMovingDirection(int threshold) {
    std::list<Direction>::iterator li = buffer_.begin();
    Direction pre = *li;
    ++li;
    int leftVote = 0;
    int rightVote = 0;
    while (li != buffer_.end()) {
      Direction cur = *li;
      if (pre == LEFT && cur == RIGHT) {
        ++rightVote;
      } else if (pre == RIGHT && cur == LEFT) {
        ++leftVote;
      }
      ++li;
    }
    if (leftVote <= threshold && rightVote <= threshold) {
      return INVALID;
    }
    std::cout << "leftVote: " << leftVote << std::endl;
    std::cout << "rightVote: " << rightVote << std::endl;
    return leftVote > rightVote ? LEFT : RIGHT;
  }

protected:
  std::list<Direction> buffer_;
  int bufferSize_;
};

class Detector {
public:
  Detector(int bufferSize) : seqAnalyzer_(bufferSize) {
    bufferSize_ = bufferSize;
  }

  Direction classifyObjectPosition(cv::Mat& frame, const double threshold);
  Direction detectMotion(cv::Mat& sum);

  void addFrame(const cv::Mat& frame);
protected:
  std::list<cv::Mat> buffer_;
  int bufferSize_;
  SequenceAnalyzer seqAnalyzer_;
};

#endif // DETECTOR_H
