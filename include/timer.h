#ifndef _timer_h
#define _timer_h
//including necessary header files
#include "capture_frame.h"
#include "view_frame.h"
#include <time.h>
/*
    Timer class is used to find the execution time of the program
*/
class Timer
{
    private:
        Logger logger;
        clock_t time_start,time_end;//Variables for time interval calculation
    public:
        float execution_time;//Output is public so that it can be accessed.

        float fps;//Output Maximum capable fps.
        
        void timer_init();//Function which starts the timer

        void timer_end();//Function to end the timer and calculate the interval

        CaptureFrame add_time(CaptureFrame original_frame);//Function to print the execution time on image
        CaptureFrame add_fps(CaptureFrame original_frame);//Function to print the maximum fps on image

};

#endif