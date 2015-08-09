#ifndef DETECTOR_H
#define DETECTOR_H
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

#include "common.h"

class SequenceAnalyzer {
public:
  SequenceAnalyzer(int bufferSize, int gridWidth);
  void addValue(Position value);
  virtual Direction detectMovingDirection(double threshold) = 0;
  double calPositionMean(const PositionIter& start, 
                         const PositionIter& end);

protected:
  std::list<int> buffer_;
  int bufferSize_;
  int gridWidth_;
  int invalidCount_;
};

class HandMovementAnalyzer : public SequenceAnalyzer {
public:
  HandMovementAnalyzer(int gridWidth);
  Direction detectMovingDirection(double  threshold);
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

  Position identifyObjectPosition2(cv::Mat& frame, 
                             const int regionNumber,
                             const double threshold);
  bool extractForeground(cv::Mat& sum);
  bool extractEdge(cv::Mat frame, cv::Mat& sum);
  Position detect(cv::Mat& foreground);

  void addFrame(const cv::Mat& frame, const float borderPercent = 0.5);
protected:
  std::list<cv::Mat> buffer_;
  int bufferSize_;
  int gridWidth_;
  double motionThreshold_;
};


#endif // DETECTOR_H
