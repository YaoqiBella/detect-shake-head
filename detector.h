#ifndef DETECTOR_H
#define DETECTOR_H
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

#include "common.h"
typedef int Position;

class SequenceAnalyzer {
public:
  SequenceAnalyzer(int bufferSize) {
    bufferSize_ = bufferSize;
  }

  void addValue(const Position& value) {
    if (value < 0) {
      return;
    }
    buffer_.push_back(value);
    if (buffer_.size() > bufferSize_) {
      buffer_.pop_front();
    }
  }

  virtual Direction detectMovingDirection(int threshold) = 0;

protected:
  std::list<int> buffer_;
  int bufferSize_;
};

class HandMovementAnalyzer : public SequenceAnalyzer {
public:
  HandMovementAnalyzer() : SequenceAnalyzer(2) {}
  Direction detectMovingDirection(int threshold) {
    std::list<int>::iterator li = buffer_.begin();
    int pre = *li, cur;
    ++li;
    int leftVote = 0;
    int rightVote = 0;
    while (li != buffer_.end()) {
      cur = *li;
      std::cout << "pre: " << pre << " cur: " << cur << std::endl;
      if (pre < cur) {
        ++rightVote;
      } else if (pre > cur) {
        ++leftVote;
      }
      pre = cur;
      ++li;
    }
    std::cout << "leftVote: " << leftVote 
              << " rightVote: " << rightVote << std::endl;
    if (leftVote < rightVote) {
      return RIGHT;
    } else if (leftVote == rightVote) {
      return INVALID;
    } else {
      return LEFT;
    }
  }


};

typedef int Position;

class MotionDetector {
public:
  MotionDetector(int bufferSize, int gridWidth, double motionThreshold) {
    bufferSize_ = bufferSize;
    gridWidth_ = gridWidth;
    motionThreshold_ = motionThreshold;
  }

  Position identifyObjectPosition(cv::Mat& frame, 
                             const int regionNumber,
                             const double threshold);
  bool extractForeground(cv::Mat& sum);
  Position detect(cv::Mat& foreground);

  void addFrame(const cv::Mat& frame);
protected:
  std::list<cv::Mat> buffer_;
  int bufferSize_;
  int gridWidth_;
  double motionThreshold_;
};


#endif // DETECTOR_H
