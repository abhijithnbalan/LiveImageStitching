#ifndef _algorithm_h
#define _algorithm_h

#include "capture_frame.h"


class Algorithm
{
  public:
    int CLAHE_clip_limit;

    CaptureFrame CLAHE_dehaze(CaptureFrame input_image); //CLAHE based basic dehazing algorithm
    cv::Mat CLAHE_dehaze(cv::Mat input_image); //CLAHE based basic dehazing algorithm
    CaptureFrame hist_equalize(CaptureFrame input_image);// histogram equilization algorithm
    void set_CLAHE_clip_limit(int clip_limit);//support function for CLAHE dehazing
    Algorithm();//The constructor
};

#endif