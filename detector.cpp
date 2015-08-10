#include <cassert>
#include <cstdlib>
#include <cmath>
#include <vector>
#include <algorithm>
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

bool approxEqual(const double val, const double target, const double tolerance) {
  return std::abs(val - target) < tolerance;
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
  if (approxEqual(mean1, 2, tolerance) && 
      (approxEqual(mean0, 0, tolerance) || 
       approxEqual(mean0, 1, tolerance))) {
    return RIGHT;
  }
  if (approxEqual(mean1, 0, tolerance) && 
      (approxEqual(mean0, 1, tolerance) || 
       approxEqual(mean0, 2, tolerance))) {
    return LEFT;
  }
  return INVALID;
  // std::cout << "mean0: " << mean0 << " mean1: " << mean1 << std::endl;
  // if (mean1 > mean0 + tolerance) {
  //   return RIGHT;
  // }
  // if (mean1 < mean0 - tolerance) {
  //   return LEFT;
  // }
  // return INVALID;

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



bool MotionDetector::extractForeground(MatIter start, MatIter end, cv::Mat& sum) {
  if (buffer_.size() != bufferSize_) {
    std::cout << "unknown buffer_ size! " << std::endl;
    return false;
  }
  cv::Mat d, pre, cur;
  MatIter li = start;
  pre = *li;
  ++li;
  while (li != end) {
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

Position MotionDetector::identifyObjectPosition(cv::Mat& frame, const double threshold) {
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
  // for (int i = 0; i < gridSepPos_.size(); ++i) {
  //   std::cout << " " << gridSepPos_[i] << std::endl;
  // }
  // std::cout << std::endl;
  yPosition /= count;
  double yPositionNormalized = yPosition / s.width;
  int yRegionNumber = std::upper_bound(gridSepPos_.begin(), gridSepPos_.end(), yPositionNormalized) - gridSepPos_.begin();
  yRegionNumber = gridWidth_.size() - yRegionNumber;
  std::cout << "count: " << count << " threshold: " << threshold
            << " yPosition: " << yPosition
            << "region: " << yRegionNumber << std::endl;
  if (count < threshold) {
    return -1;
  }
  return yRegionNumber;
}

Position MotionDetector::identifyObjectPositionWithMotionDiff(cv::Mat& frame, 
                                                              const double threshold) {
  cv::Size s = frame.size();
  int gridWidthLen = gridWidth_.size();
  std::vector<double> lightness(gridWidthLen, 0);

  double maxLight = -1, sumLight = 0;
  int maxLightRegion = -1;

  for (int i = 0; i < gridWidthLen; ++i) {
    int yStart = (1 - gridSepPos_[i+1]) * s.width;
    int yEnd = (1 - gridSepPos_[i]) * s.width;
    cv::Mat regionMat = frame(cv::Range::all(), 
                              cv::Range(yStart, yEnd));
    lightness[i] = cv::sum(regionMat).val[0];
    if (lightness[i] > maxLight) {
      maxLight = lightness[i];
      maxLightRegion = i;
    }
    sumLight += lightness[i];
  }
  std::cout << "maxLight / sumLight: " << maxLight / sumLight;
  std::cout << "maxLightRegion: " << maxLightRegion<< std::endl;
  std::cout<< "threshold / gridWidth: " <<  threshold / gridWidthLen << std::endl;
  if (maxLight / sumLight  <  threshold / gridWidthLen) {
    return -1;
  }
  return maxLightRegion;
}


Position MotionDetector::majorityVote(std::vector<Position> votes) {
  int gridWidthLen = gridWidth_.size();
  std::vector<int> vote(gridWidthLen, 0);
  for (int i = 0; i < votes.size(); ++i) {
    Position v = votes[i];
    assert(v < gridWidthLen);
    if (v >= 0) {
      vote[v]++;
    }
  }
  int maxVote = 0, maxVotePosition = -1;
  for (int i = 0; i < gridWidthLen; ++i) {
    if (vote[i] > maxVote) {
      maxVote = vote[i];
      maxVotePosition = i;
    }
  }

  return maxVote < (votes.size() + 1) / 2 ? -1 : maxVotePosition;

  // int maxVotePositionCount = 0;
  // for (int i = 0; i < gridWidthLen; ++i) {
  //   if (vote[i] == maxVote) {
  //     ++maxVotePositionCount;
  //   }
  // }
  // if (maxVotePositionCount > 1) { // there is a draw
  //   return -1;
  // } else if (maxVote < 2){
  //   return maxVotePosition;
  // }
}

Position MotionDetector::detect(cv::Mat& res) {
  std::vector<cv::Mat> buffer(buffer_.begin(), buffer_.end());
  int N = buffer.size();
  cv::Size frameSize = buffer[N-1].size();

  std::vector<Position> votes;
  Position p;

  // Method: identify region using three frame difference of detected edges.
  cv::Mat foreground(frameSize.height, frameSize.width, CV_8UC1);
  foreground.setTo(cv::Scalar(0));
  extractForeground(buffer.end() - 4, buffer.end(), foreground);

  cv::Mat edge;
  extractEdge(buffer[N-1], edge);

  cv::Mat motionDiff1;
  cv::bitwise_and(foreground, edge, motionDiff1);
  p = identifyObjectPosition(motionDiff1, motionThreshold_);
  votes.push_back(p);

  // Method: identify region using sum of multiple frame difference and
  // edge detection.
  cv::Mat foreground2(frameSize.height, frameSize.width, CV_8UC1);
  foreground2.setTo(cv::Scalar(0));
  extractForeground(buffer.begin(), buffer.end(), foreground2);

  cv::Mat motionDiff2;
  cv::bitwise_and(foreground2, edge, motionDiff2);
  p = identifyObjectPosition(motionDiff2, motionThreshold_);
  votes.push_back(p);

  res = motionDiff2;

  // Method: identify region using only frame difference.
  p = identifyObjectPositionWithMotionDiff(foreground, 1.2);
  votes.push_back(p);
  Position voteReuslt = majorityVote(votes);

  std::cout << "reuslt: ";
  for (int i = 0; i < votes.size(); ++i) {
    std::cout << " " << votes[i];
  }
  std::cout << " vote result: " << voteReuslt << std::endl;
  return voteReuslt;
}
