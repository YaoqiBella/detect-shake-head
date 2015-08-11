#ifndef DETECTOR_H
#define DETECTOR_H
#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "common.h"

typedef int Position;

// Class that detects the position of moving object. 
//
// It divides the
// horizontal-axis to several regions (i.e., dividing the screen to a 1xN grid).
// The class only reports which regions the moving object is in. Several
// methods are applied and then the result is returned through majority
// vote. For each method, there are two steps:
// 1. find the moving object through image processing.
// 2. estimate or classify the right position through analyzing the output
//    of image processing.
// 
// Constructor arguments:
//   bufferSize: int. The number of frames used for each detection.
//   gridWidth: a vector of double. The width (in percentage) of each region. The sum of
//   all elements in gridWidth should be equal to 1. 
class MotionDetector {
public:
  MotionDetector(int bufferSize, std::vector<double> gridWidth);

  // Identify the position of the moving object using the center of the
  // detected moving edge.
  // 
  //
  // Arguments:
  //   frame: cv::Mat, the input frame containing the image processing result.
  //   min_point_in_contour: a threshold to determine whether a position
  //   should be returned.   
  // Returns:
  //   Estimated position. Returns the region that the center belongs to
  //   if the # of points in the detected moving edge is >=
  //   min_point_in_contour, and -1 otherwise.
  Position identifyObjectPositionByCenterOfMovingEdge(cv::Mat& frame, const double min_point_in_contour);

  // Identify the position of the moving object using the strengh of
  // motion.
  //
  // Arguments:
  //   frame: cv::Mat, the input frame containing the image processing result.
  //   min_strength_fractor: a threshold to determine whether a position
  //   should be reported.
  // Returns:
  //   Estimated position. Returns the region of strongest strength if
  //   maxStrength / sumStrength <  min_strength_fractor / gridWidthLen,
  //   and returns -1 otherwise.
  Position identifyObjectPositionByWithMotionStrength(cv::Mat& frame, 
                                                      const double min_strength_fractor);
  // Majority vote of several returned positions.
  //
  // Arguments:
  //   votes: a vector of Position, the position returned by different
  //   methods.
  // Returns:
  //   The position with  more than half of the
  //   votes if it exists, and -1 otherwise.
  Position majorityVote(std::vector<Position> votes);


  // Extract foreground from a sequence of image frames.
  //
  // Arguments:
  //   start: the iterator for the first frame.
  //   end: the iterator of the frame that is the next frame of the last
  //   frame, i.e., the range is [start, end).
  //   sum: the reference variable that the returned frame will be written to.
  // Returns: 
  //   bool: true if foreground is extracted.
  bool extractForeground(MatIter start, MatIter end, cv::Mat& sum);

  // Extract edge from a frame.
  //
  // Arguments:
  //   frame: the input frame
  //   sum: the reference variable  that the returned edge will be
  //   written to.
  // Returns:
  //   bool: true if edge is extracted.
  bool extractEdge(cv::Mat frame, cv::Mat& sum);

  // Detect the position of moving object.
  //
  // It applies several methods to the frames in buffer_ and returns
  // result by majority vote.
  Position detect();

  // Add frame to the buffer.
  //
  // It will make sure the # of frames in buffer_ is no more than
  // bufferSize_;
  //
  // Arguments:
  //   frame: frame that will be added.
  void addFrame(const cv::Mat& frame);
protected:
  std::list<cv::Mat> buffer_;
  int bufferSize_;
  std::vector<double> gridWidth_;
  std::vector<double> gridSepPos_;
};


// Class that analyze the positions of moving objects and reports the
// right command.
//
// Constructor Arguments:
//   bufferSize: int, the number of positions used for detecting
//   command.
//   gridWidth: int, the # of grids in horizontal direction.
class SequenceAnalyzer {
public:
  SequenceAnalyzer(int bufferSize, int gridWidth);

  // Add value to the buffer_. It will make sure # of values is no more
  // than bufferSize_;
  void addValue(Position value);

  // Detect command issued by the user.
  //
  // Arguments:
  //   tolerance: the tolerance used to detect whether use has issued a
  //   command or not.
  //
  // Returns:
  //   Command. The detected command.
  Command detectCommand(double tolerance);

  // Reduce noise in the sequence.
  //
  // It remove noise using a simple rule: if x[i-1] == x[i+1] != x[i],
  // then change x[i] to x[i-1]. For example, 0100111 will be corrected
  // as 0000111. The reason is that user's command usually last more
  // than one frame, so rapid change is likely due to noise.
  //
  // Arguments:
  //  seq: reference variable to a vector of Position, input position
  //  sequence. The sequence is changed in place.
  // Returns:
  //  int. The # of codes corrected.
  int codeCorrection(std::vector<Position>& seq);

  // Calculate the mean for a sequence of position.
  //
  // Arguments:
  //   start: the start iterator
  //   end: the iterator after the last element. i.e., the range is
  //   [start, end)
  // Returns:
  //   double: the mean of position. Positions are converted to ints.
  double calPositionMean(const PositionIter& start, 
                         const PositionIter& end);

protected:
  std::list<int> buffer_;
  int bufferSize_;
  int gridWidth_;
  int invalidCount_;
};

#endif // DETECTOR_H
