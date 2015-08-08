#ifndef ANIMATION_H
#define ANIMATION_H
#include "opencv2/highgui/highgui.hpp"
#include <iostream>

#include "common.h"

using namespace cv;
using namespace std;


class Animator {
public:
  Animator(int frameRate) {
    frameRate_ = frameRate;
  }
  virtual void animate(Mat& img, const double degree) = 0;
  bool addAnimateStartFromNow(double duration);
  void playFrame(Mat& img, bool show);
protected:
  int totalFrameNum_;
  int currentFrameNum_;
  int frameRate_;
};

class ArrowAnimator : public Animator{
public:
  ArrowAnimator(int frameRate) : Animator(frameRate) {
    moveTo_ = INVALID;
  }

  static void drawArrow(Mat& img, const Direction direction, const Point& center, cv::Scalar color);
  void animateArrow(Mat& img, const Direction moveTo, const double degree);

  bool addAnimateStartFromNow(double duration, const Direction moveTo, cv::Scalar color) {
    if (currentFrameNum_ < totalFrameNum_) {
      return false;
    }

    moveTo_ = moveTo;
    color_ = color;
    return Animator::addAnimateStartFromNow(duration);
  }

  void animate(Mat& img, const double degree) {
    animateArrow(img, moveTo_, degree);
  }
protected:
  Direction moveTo_;
  cv::Scalar color_;
};

#endif // ANIMATION_H
