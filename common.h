#ifndef COMMON_H
#define COMMON_H

enum Direction {
  INVALID = -1,
  RIGHT = 0,
  DOWN = 1,
  LEFT = 2,
  UP = 3,
};

typedef int Position;
typedef std::vector<Position>::iterator PositionIter;
typedef std::vector<cv::Mat>::iterator MatIter;

#endif // COMMON_H

