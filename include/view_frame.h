
#ifndef _view_frame_h
#define _view_frame_h
//Including necessary header files
#include "capture_frame.h"
#include "logger.h"
#include <opencv2/opencv.hpp> 
#include <stdio.h>
#include <string.h>

/*
    ViewFrame class is used to show the output i.e. either image or video.
    Multiple modes of viewing is provided.
*/
class ViewFrame
{  
    private://These variables will only be used internally.
        cv::Mat image1,image2,image3,image4;Logger logger;

    public:
        //Single Input
        void single_view_interrupted(CaptureFrame image);//Shows a single output and wait for userkey to continue
        void single_view_interrupted(CaptureFrame image,int resize_percentage);//Overloaded function which also resizes the output
  
        void single_view_uninterrupted(CaptureFrame image);//Shows output and continues. Used inside loops.
        void single_view_uninterrupted(CaptureFrame image,int resize_percentage );//Overloaded function which also resizes the output

        //Multiple outputs
        void multiple_view_interrupted(CaptureFrame image_1, CaptureFrame image_2 );//Shows two images side by side and waits for user input.
        void multiple_view_interrupted( CaptureFrame image_1,CaptureFrame image_2 ,int resize_percentage);//Overloaded function which also resizes the output
        void multiple_view_interrupted(CaptureFrame image_1 ,CaptureFrame image_2 ,CaptureFrame image_3);//Overloaded function to shows three images 
        void multiple_view_interrupted( CaptureFrame image_1 ,CaptureFrame image_2 ,CaptureFrame image_3 ,int resize_percentage);//Overloaded function which also resizes the output
        void multiple_view_interrupted(CaptureFrame image_1 ,CaptureFrame image_2 ,CaptureFrame image_3 ,CaptureFrame image_4);//Overloaded function to show four outputs.
        void multiple_view_interrupted( CaptureFrame image_1 ,CaptureFrame image_2 ,CaptureFrame image_3 ,CaptureFrame image_4 ,int resize_percentage);//Overloaded function which also resizes the output

        //The following 3 functions are the overloaded functions of the above modified for usage in loops.
        void multiple_view_uninterrupted(CaptureFrame image_1 ,CaptureFrame image_2 );//2 Images
        void multiple_view_uninterrupted( CaptureFrame image_1 ,CaptureFrame image_2 ,int resize_percentage );//Overloaded function which also resizes the output
        void multiple_view_uninterrupted(CaptureFrame image_1 ,CaptureFrame image_2 ,CaptureFrame image_3 );//3 Images
        void multiple_view_uninterrupted( CaptureFrame image_1 ,CaptureFrame image_2 ,CaptureFrame image_3 ,int resize_percentage);//Overloaded function which also resizes the output
        void multiple_view_uninterrupted(CaptureFrame image_1 ,CaptureFrame image_2 ,CaptureFrame image_3 ,CaptureFrame image_4 );//4 Images
        void multiple_view_uninterrupted( CaptureFrame image_1 ,CaptureFrame image_2 ,CaptureFrame image_3 ,CaptureFrame image_4 ,int resize_percentage);//Overloaded function which also resizes the output
        
        // Overloaded Functions for overlaying data onto the image
        //in percentage
        CaptureFrame add_overlay_percent(CaptureFrame input_image, int x_percentage, int y_percentage, int data);
        CaptureFrame add_overlay_percent(CaptureFrame, int x_percentage , int y_percentage , std::string data,cv::Scalar color ,float font_size ,int thickness);
        CaptureFrame add_overlay_percent(CaptureFrame image, int x_percentage , int y_percentage , float data);
        cv::Mat add_overlay_percent(cv::Mat image, int x_percentage , int y_percentage , std::string data ,cv::Scalar color ,float font_size ,int thickness);

        //When actual point is known
        CaptureFrame add_overlay(CaptureFrame, int x_point , int y_point , int data);
        CaptureFrame add_overlay(CaptureFrame, int x_point , int y_point , std::string data);
        CaptureFrame add_overlay(CaptureFrame image, cv::Rect overlay_rectangle, cv::Mat overlay_image);

        CaptureFrame join_image_horizontal(CaptureFrame imge1,CaptureFrame image2,int mode);
        CaptureFrame join_image_vertical(CaptureFrame imge1,CaptureFrame image2,int mode);

};

#endif