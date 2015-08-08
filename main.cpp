#include <iostream>
#include "opencv2/opencv.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "animation.h"
#include "detector.h"

using namespace cv;
using namespace std;


int main(int argc, char* argv[])
{
  VideoCapture cap(0); // open the video camera no. 0
  cout << "Video camera opened";

  if (!cap.isOpened())  // if not success, exit program
  {
    cout << "Cannot open the video cam" << endl;
    return -1;
  }

  double dWidth = cap.get(CV_CAP_PROP_FRAME_WIDTH); //get the width of frames of the video
  double dHeight = cap.get(CV_CAP_PROP_FRAME_HEIGHT); //get the height of frames of the video

  cout << "Frame size : " << dWidth << " x " << dHeight << endl;

  namedWindow("MyVideo",CV_WINDOW_AUTOSIZE); //create a window called "MyVideo"

  int frameRate = 35;

  int i = 0;
  ArrowAnimator arrowAnimator(frameRate);
  MotionDetector detector(5, 2, 1.2);
  HandMovementAnalyzer handMovementAnalyzer;
  Mat frame;
  while (1)
  {
    i++;

    bool bSuccess = cap.read(frame); // read a new frame from video
    Size s = frame.size();
    Mat grayFrame(s.height, s.width, CV_8UC1);
    cvtColor(frame, grayFrame, CV_BGR2GRAY);

    if (!bSuccess) //if not success, break loop
    {
      cout << "Cannot read a frame from video stream" << endl;
      break;
    }

    detector.addFrame(grayFrame);
    Mat foreground(s.height, s.width, CV_8UC1);
    foreground.setTo(Scalar(0));
    Position position = detector.detect(foreground);
    handMovementAnalyzer.addValue(position);
    Direction direction  = handMovementAnalyzer.detectMovingDirection(1);
    if (direction != INVALID) {
      arrowAnimator.addAnimateStartFromNow(0.1, direction);
    }
    Mat flippedFrame;
    flip(frame, flippedFrame, 1);
    arrowAnimator.playFrame(flippedFrame, true);
    // imshow("MyVideo", res); //show the frame in "MyVideo" window

    if (waitKey(1000 / frameRate) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
    {
      cout << "esc key is pressed by user" << endl;
      break; 
    }
  }
  return 0;

}
