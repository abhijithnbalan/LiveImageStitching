#ifndef _image_processing_h
#define _image_processing_h

//Including the essential header files
#include <opencv2/opencv.hpp> 
#include "capture_frame.h"
#include "algorithm.h"

/*
    ImageProcessing class will be used for all processing operations on the image or image steam.
*/
class ImageProcessing //-------medianBlur or CLAHE dehazing-------//
{
    private://These variables will be used internally
        cv::Mat image_hsv,inter,image_hsv_threshold,image_hsv_threshold_low,image_hsv_threshold_high,image_hsv_threshold_white;
        int x,y,width,height;
        //The following varibles(preset by Constructor) can me modified.
        
    protected://these variables will be shared in inheritance
        cv::Rect roi; //Region of interest
        bool flag;//
        int hue_upper,hue_lower,saturation_upper,value_lower,lightness_upper;//thresholding values (private)
        

    public:
    int white_use_value;
        bool use_white;//controlling variable for white filtering
        Algorithm algo;//algorithm object
        //threshold(public) can be changed by user
        cv::Scalar thresh_high_0,thresh_high_180,thresh_low_0,thresh_low_180,thresh_white;
        //element for dilation and Morphological Opening.
        cv::Mat element;
        CaptureFrame resize_image(CaptureFrame,int percent);
        CaptureFrame roi_selection(CaptureFrame input_image);//Selecting the Region of interst.

        cv::Mat roi_selection(cv::Mat input_image);//Selecting the Region of interst.

        CaptureFrame image_segmentation(CaptureFrame input_image);//Color segmentation. according to threshold set.

        cv::Mat image_segmentation(cv::Mat input_image);//Color segmentation. according to threshold set.

        void set_threshold(CaptureFrame current_scene_image);//Function to set the threshold value according to water type.

        void set_threshold(int Hue_Low, int Hue_High,int Saturation_High, int Saturation_Low, int Value_High, int Value_Low);//Function to set the threshold value according to water type.

        void set_roi(int x_percent, int y_percent, int width, int height);// Function to set the Region of Interest 


        static void on_trackbar(int trackbar_value ,void* trackbar_user_pointer);//Callback function for trackbar
        static void on_trackbar_single(int trackbar_value ,void* trackbar_user_pointer);//Callback Function for trackbar single laser
        void myhandler(int trackbar_value);//function to change parameter for trackbar
        void myhandler_single(int trackbar_value);//function to change parameter for trackbar single laser
        static void on_button(int trackbar_value,void* trackbar_user_pointer);//Callback Function for button
        void myhandlerbutton(int trackbar_value);//function to change parameter for use_white button

        ImageProcessing();//Constructor definition The values are preset in this constructor.

};

#endif