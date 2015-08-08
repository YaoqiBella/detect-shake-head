#include <cmath>
#include <vector>
#include "detector.h"

SequenceAnalyzer::SequenceAnalyzer(int bufferSize, int gridWidth) {
  bufferSize_ = bufferSize;
  gridWidth_ = gridWidth;
  invalidCount_ = 0;
}


void SequenceAnalyzer::addValue(Position value) {
  if (value < 0) {
    ++invalidCount_;
    return;
  }
  invalidCount_ = 0;
  buffer_.push_back(value);
  if (buffer_.size() > bufferSize_) {
    buffer_.pop_front();
  }
}

double SequenceAnalyzer::calPositionMean(const PositionIter& start, 
                                             const PositionIter& end) {
  PositionIter li = start;
  int count = 0;
  double sum = 0;
  while (li != end) {
    sum += *li;
    ++count;
    ++li;
  }
  return sum / count;
}

HandMovementAnalyzer::HandMovementAnalyzer(int gridWidth) : SequenceAnalyzer(4, gridWidth) {}

Direction HandMovementAnalyzer::detectMovingDirection(double tolerance) {
  std::vector<Position> seq(buffer_.begin(), buffer_.end());
  int N = seq.size();
  std::cout << "invalidCount_: " << invalidCount_ << std::endl;
  if (N < 2 || invalidCount_ > 10) {
    return INVALID;
  }
  
  double mean0 = calPositionMean(seq.begin(), seq.begin() + N / 2);
  double mean1 = calPositionMean(seq.begin() + N / 2, seq.begin() + N);
  std::cout << "mean0: " << mean0 << " mean1: " << mean1 << std::endl;
  if (mean1 > mean0 + tolerance) {
    return RIGHT;
  }
  if (mean1 < mean0 - tolerance) {
    return LEFT;
  }
  return INVALID;

}


HeadMovementAnalyzer::HeadMovementAnalyzer(int gridWidth) : SequenceAnalyzer(12, gridWidth) {
}



Direction HeadMovementAnalyzer::detectMovingDirection(double tolerance) {
  std::vector<Position> seq(buffer_.begin(), buffer_.end());
  int N = seq.size();
  if (N < 3 || invalidCount_ > 10) {
    return INVALID;
  }
  
  double mean0 = calPositionMean(seq.begin(), seq.begin() + N / 3);
  double mean1 = calPositionMean(seq.begin() + N / 3, seq.begin() + N / 3 * 2);
  double mean2 = calPositionMean(seq.begin() + N / 3 * 2, seq.begin() + N);
  if (mean1 > mean0 + tolerance && mean1 > mean2 + tolerance) {
    return RIGHT;
  }
  if (mean1 < mean0 - tolerance && mean1 < mean2 - tolerance) {
    return LEFT;
  }
  return INVALID;
}



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
  std::cout << "maxLight / sumLight: " << maxLight / sumLight;
  std::cout << "maxLightRegion: " << maxLightRegion<< std::endl;
  std::cout<< "threshold / gridWidth: " <<  threshold / gridWidth << std::endl;
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
