#ifndef COMMON_H
#define COMMON_H

enum Command {
  INVALID = -1,
  RIGHT = 0,
  DOWN = 1, // Not implemented.
  LEFT = 2,
  UP = 3, // Note implemented.
};

typedef Command Direction;

typedef int Position;
typedef std::vector<Position>::iterator PositionIter;
typedef std::vector<cv::Mat>::iterator MatIter;

#endif // COMMON_H

