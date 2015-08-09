#include <cassert>
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

HandMovementAnalyzer::HandMovementAnalyzer(int gridWidth) : SequenceAnalyzer(6, gridWidth) {}


int HandMovementAnalyzer::codeCorrection(std::vector<Position>& seq) {
  int N = seq.size();
  if (N < 3) {
    return 0 ;
  }
  int count = 0;
  for (int i = 1; i < N-1; ++i) {
    if (seq[i-1] == seq[i+1] && seq[i] != seq[i-1]) {
      seq[i] = seq[i-1];
      ++count;
    }
  }
  return count;
}

Direction HandMovementAnalyzer::detectMovingDirection(double tolerance) {
  std::vector<Position> seq(buffer_.begin(), buffer_.end());
  int correctCount = codeCorrection(seq);
  std::cout << "correctCount: " << correctCount << std::endl;
  int N = seq.size();
  std::cout << "invalidCount_: " << invalidCount_ << std::endl;
  if (N < 2 || invalidCount_ > 3) {
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

void MotionDetector::addFrame(const cv::Mat& frame, const float borderPercent) {
  cv::Mat realFrame;
  cv::Size s = frame.size();
  int borderLength = (int) s.width * borderPercent;
  cv::hconcat(frame(cv::Range::all(), cv::Range(0, borderLength)),
      frame(cv::Range::all(), cv::Range(s.width - borderLength, s.width)),
      realFrame);
  buffer_.push_back(realFrame);
  if (buffer_.size() > bufferSize_) {
    buffer_.pop_front();
  }
}


bool MotionDetector::extractForeground(cv::Mat& sum) {
  if (buffer_.size() != bufferSize_) {
    std::cout << "unknown buffer_ size! " << std::endl;
    return false;
  }
  cv::Mat d, pre, cur;
  std::list<cv::Mat>::iterator li = buffer_.begin();
  pre = *li;
  ++li;
  while (li != buffer_.end()) {
    cur = *li;
    cv::absdiff(pre, cur, d);
    cv::add(sum, d, sum);
    pre = *li;
    ++li;
  }
  return true;
}

bool MotionDetector::extractEdge(cv::Mat frame, cv::Mat& edge) {
  int lowThreshold = 30;
  int ratio = 3;
  int kernelSize = 3;
  // Reduce noise with a kernel 3x3
  cv::blur(frame, edge, cv::Size(kernelSize, kernelSize));

  std::cout << "run here" << std::endl;
  // Canny detector
  cv::Canny(edge, edge, lowThreshold, lowThreshold * ratio, kernelSize);
  std::cout << "run here 2" << std::endl;

  return true;
}

Position MotionDetector::identifyObjectPosition(cv::Mat& frame, const int gridWidth, const double threshold) {
  cv::Size s = frame.size();
  double yPosition = 0;
  int count = 0;
  for (int i = 0; i < s.height; ++i) {
    for (int j = 0; j < s.width; ++j) {
      if (static_cast<int>(frame.at<uchar>(i, j)) > 128 ) {
        yPosition += j;
        ++count;
      }
    }
  }
  yPosition /= count;
  int yReigionNumber = (int) (s.width - yPosition) / (s.width / gridWidth);
  std::cout << "count: " << count << " threshold: " << threshold
            << " yPosition: " << yPosition
            << "region: " << yReigionNumber << std::endl;
  if (count < threshold) {
    return -1;
  }
  return yReigionNumber;
}

Position MotionDetector::identifyObjectPositionWithMotionDiff(cv::Mat& frame, const int gridWidth, const double threshold) {
  cv::Size s = frame.size();
  std::vector<double> lightness(gridWidth, 0);
  int regionWidth = s.width / gridWidth_;

  double maxLight = -1, sumLight = 0;
  int maxLightRegion = -1;
  std::cout << "run in identifier" << std::endl;
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

Position MotionDetector::detect(cv::Mat& res) {
  cv::Size frameSize = buffer_.back().size();
  cv::Mat foreground(frameSize.height, frameSize.width, CV_8UC1);
  foreground.setTo(cv::Scalar(0));
  extractForeground(foreground);

  cv::Mat edge;
  extractEdge(buffer_.back(), edge);

  cv::bitwise_and(foreground, edge, res);

  std::vector<int> vote(2, 0);
  Position p1 = identifyObjectPosition(res, gridWidth_, motionThreshold_);
  assert(p1 < 2);
  if (p1 >= 0) {
    vote[p1]++;
  }

  Position p2 = identifyObjectPosition(edge, gridWidth_, motionThreshold_);
  assert(p2 < 2);
  if (p2 >= 0) {
    vote[p2]++;
  }

  Position p3 = identifyObjectPositionWithMotionDiff(foreground, gridWidth_, 1.3);
  assert(p3 < 2);
  if (p3 >= 0) {
    vote[p3]++;
  }
  std::cout << "p1: " << p1 << " p2: " << p2 << " p3: " << p3 << std::endl;

  if (vote[0] == vote[1]) {
    return -1;
  } else if (vote[0] < vote[1]) {
    return 1;
  } else {
    return 0;
  }
}
