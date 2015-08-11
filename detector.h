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
  Direction detectMovingDirection(double  threshold);
  int codeCorrection(std::vector<Position>& seq);
  double calPositionMean(const PositionIter& start, 
                         const PositionIter& end);

protected:
  std::list<int> buffer_;
  int bufferSize_;
  int gridWidth_;
  int invalidCount_;
};

typedef int Position;

class MotionDetector {
public:
  MotionDetector(int bufferSize, std::vector<double> gridWidth, double motionThreshold) : 
    gridSepPos_(gridWidth.size() + 1, 0) {

    bufferSize_ = bufferSize;
    gridWidth_ = gridWidth;
    motionThreshold_ = motionThreshold;

    // calculate the position of the border of to adjcent grids.
    for (int i = 1; i < gridWidth_.size() + 1; ++i) {
      gridSepPos_[i] = gridSepPos_[i-1] + gridWidth_[i-1];
    }
  }

  Position identifyObjectPosition(cv::Mat& frame, 
                             const double threshold);

  Position identifyObjectPositionWithMotionDiff(cv::Mat& frame, 
                                                const double threshold);
  Position majorityVote(std::vector<Position> votes);


  bool extractForeground(MatIter start, MatIter end, cv::Mat& sum);
  bool extractEdge(cv::Mat frame, cv::Mat& sum);
  Position detect(cv::Mat& foreground);

  void addFrame(const cv::Mat& frame, const float borderPercent = 0.5);
protected:
  std::list<cv::Mat> buffer_;
  int bufferSize_;
  std::vector<double> gridWidth_;
  std::vector<double> gridSepPos_;
  double motionThreshold_;
};


#endif // DETECTOR_H
