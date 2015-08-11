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
  double gridWidthArr[] = {0.4, 0.2, 0.4};
  int gridWidthLen = sizeof(gridWidthArr) / sizeof(double);
  MotionDetector motionDetector(8, vector<double>(gridWidthArr, gridWidthArr + gridWidthLen));
  SequenceAnalyzer sequenceAnalyzer(4, gridWidthLen);

  cv::Mat frame;
  while (1)
  {
    i++;

    bool bSuccess = cap.read(frame); // read a new frame from video
    Size s = frame.size();
    // Preprocess the images.
    Mat grayFrame(s.height, s.width, CV_8UC1);
    cvtColor(frame, grayFrame, CV_BGR2GRAY);

    if (!bSuccess) //if not success, break loop
    {
      cout << "Cannot read a frame from video stream" << endl;
      break;
    }

    motionDetector.addFrame(grayFrame);
    Position handPosition = motionDetector.detect();
    sequenceAnalyzer.addValue(handPosition);
    Command cmd = sequenceAnalyzer.detectCommand(0.2);
    if (cmd != INVALID) {
      arrowAnimator.addAnimateStartFromNow(0.2, cmd, CV_RGB(255, 255, 255));
    }
  
    Mat flippedFrame;
    flip(frame, flippedFrame, 1);
    arrowAnimator.playFrame(flippedFrame, true);

    if (waitKey(1000 / frameRate) == 27) //wait for 'esc' key press for 30ms. If 'esc' key is pressed, break loop
    {
      cout << "esc key is pressed by user" << endl;
      break; 
    }
  }
  return 0;

}
