
//including the necessary header files
#include "image_processing.h"
#include "capture_frame.h"
#include "timer.h"
#include "view_frame.h"

#include <opencv2/opencv.hpp>

CaptureFrame ImageProcessing::roi_selection(CaptureFrame object1) //Selecting the Region of interst. full width is taken.
{
    cv::Mat image1 = object1.retrieve_image().clone();
    cv::Mat temp = roi_selection(image1);
    CaptureFrame roi_image(temp, "region_of_interest");
    return roi_image;
}

//roi selection with image input
cv::Mat ImageProcessing::roi_selection(cv::Mat image1) //Selecting the Region of interst for an image file . full width is taken.
{
    int roi_x = image1.cols*x/100 -image1.cols*width/200;
    int roi_y = image1.rows*y/100 -image1.rows*height/200;
    int roi_width = image1.cols*width/100;
    int roi_height = image1.rows*height/100;
    roi = cv::Rect(roi_x ,roi_y ,roi_width ,roi_height);

    cv::Mat temp = image1(roi).clone();
    return temp;
}

CaptureFrame ImageProcessing::image_segmentation(CaptureFrame object1) //Color segmentation. according to threshold set.
{
    cv::Mat segmented;
    segmented = image_segmentation(object1.retrieve_image());
    CaptureFrame hsv_threshold(segmented, "Segmented Image");
    return hsv_threshold;
}

cv::Mat ImageProcessing::image_segmentation(cv::Mat object1) //Color segmentation. according to threshold set.
{
    cv::Mat image_hls;
    cvtColor(object1, image_hsv, cv::COLOR_BGR2HSV); //Convert to HSV format for color identification
    cvtColor(object1, image_hls, cv::COLOR_BGR2HLS); //Convert to HLS format for white color filtering.
    
    // Segmentation according to the value set in threshold variables
    inRange(image_hsv, thresh_low_0, thresh_high_0, image_hsv_threshold_low);
    inRange(image_hsv, thresh_low_180, thresh_high_180, image_hsv_threshold_high);
    image_hsv_threshold = image_hsv_threshold_low + image_hsv_threshold_high;
    
    if (use_white)//white is filtered in hsl image format and filter using lightness value
    {
        inRange(image_hls, thresh_white, cv::Scalar(255, 255, 255, 0), image_hsv_threshold_white);
        image_hsv_threshold = image_hsv_threshold + image_hsv_threshold_white;
    }
    //Morphological Transformations for noise reduction.
    morphologyEx(image_hsv_threshold, image_hsv_threshold, cv::MORPH_OPEN, element, cv::Point(-1, -1), 2);
    dilate(image_hsv_threshold, image_hsv_threshold, element, cv::Point(-1, -1), 3);//dilating 3 times
    return image_hsv_threshold;
}
void ImageProcessing::set_threshold(CaptureFrame object1) //Function to set the threshold value according to water type.
{
    //Conditions and set the threshold
    // //----Green---//
    //     thresh_low_0 = cv::Scalar(45, 160, 180, 0),thresh_low_180 = cv::Scalar(70, 255, 255, 0),
    //     thresh_high_0 = cv::Scalar(180, 255, 255, 0), thresh_high_180 = cv::Scalar(180, 255, 255, 0);
    // //-----Blue-----//
    //     thresh_low_0 = cv::Scalar(82, 160, 180, 0),thresh_low_180 = cv::Scalar(130, 255, 255, 0),
    //     thresh_high_0 = cv::Scalar(180, 255, 255, 0), thresh_high_180 = cv::Scalar(180, 255, 255, 0);
    // //----White---//
    //     thresh_low_0 = cv::Scalar(0, 0, 180, 0),thresh_low_180 = cv::Scalar(180, 20, 255, 0),
    //     thresh_high_0 = cv::Scalar(180, 255, 255, 0), thresh_high_180 = cv::Scalar(180, 255, 255, 0);
    // //----Yellow----//
    //     thresh_low_0 = cv::Scalar(21, 160, 180, 0),thresh_low_180 = cv::Scalar(33, 255, 255, 0),
    //     thresh_high_0 = cv::Scalar(180, 255, 255, 0), thresh_high_180 = cv::Scalar(180, 255, 255, 0);
    //------RED------//
    thresh_low_0 = cv::Scalar(0, 160, 180, 0), thresh_high_0 = cv::Scalar(16, 255, 255, 0),
    thresh_low_180 = cv::Scalar(160, 160, 180, 0), thresh_high_180 = cv::Scalar(180, 255, 255, 0);
}
//function to set threshold according to the 6 values supplied by user.
void ImageProcessing::set_threshold(int Hue_Low, int Hue_High,int Saturation_High, int Saturation_Low, int Value_High, int Value_Low)
{
    if(Hue_Low > Hue_High)//only for red
    {
        thresh_low_0 = cv::Scalar(0, Saturation_Low, Value_Low, 0), thresh_low_180 = cv::Scalar(Hue_High, Saturation_High, Value_High, 0),
        thresh_high_0 = cv::Scalar(Hue_Low, Saturation_Low, Value_Low, 0), thresh_high_180 = cv::Scalar(180, Saturation_High, Value_High, 0);
    }
    else//for all the colors
    {
        thresh_low_0 = cv::Scalar(Hue_Low, Saturation_Low, Value_Low, 0), thresh_low_180 = cv::Scalar(Hue_High, Saturation_High, Value_High, 0),
        thresh_high_0 = cv::Scalar(180, 255, 255, 0), thresh_high_180 = cv::Scalar(180, 255, 255, 0);
    }
    return;
}

//function to set region of interest according to the values user inputs
void ImageProcessing::set_roi(int x_input,int y_input,int width_input,int height_input) 
{
    //theses data will be used to create Rectangular element.
    x = x_input;
    y = y_input;
    width = width_input;
    height = height_input;
    return;
}
//Callback function for trackbars
void ImageProcessing::on_trackbar(int red, void *ptr)
{
    //This pointer is passed to change parameters within class
    ImageProcessing *c = (ImageProcessing *)(ptr);
    c->myhandler(red);
    return;
}
//Callback function for trackbars in single laser ranging enabled
void ImageProcessing::on_trackbar_single(int red, void *ptr)
{
    //this pointer is passed to change the parameter
    ImageProcessing *c = (ImageProcessing *)(ptr);
    c->myhandler_single(red);
    return;
}
CaptureFrame ImageProcessing::resize_image(CaptureFrame image,int percent)
{
    cv::Mat img;
    resize(image.retrieve_image(), img, cv::Size(image.retrieve_image().cols * percent/ 100, image.retrieve_image().rows * percent / 100));
    return CaptureFrame(img,image.window_name + " resized");

}
//Function to change parameter for trackbar change
void ImageProcessing::myhandler(int red)
{
    //getting all the trackbar position and update the threshold values
    thresh_high_0 = cv::Scalar(cv::getTrackbarPos("Hue Upper threshold", "Control Panel"), 255, 255, 0);
    thresh_low_0 = cv::Scalar(0, 255 - cv::getTrackbarPos("Saturation Lower threshold", "Control Panel"), 255 - cv::getTrackbarPos("Value Lower threshold", "Control Panel"), 0);
    thresh_low_180 = cv::Scalar(180 - cv::getTrackbarPos("Hue Lower threshold", "Control Panel"), 255 - cv::getTrackbarPos("Saturation Lower threshold", "Control Panel"), 255 - cv::getTrackbarPos("Value Lower threshold", "Control Panel"), 0);
    thresh_high_180 = cv::Scalar(180, 255, 255, 0);
    thresh_white = cv::Scalar(0, 255 - cv::getTrackbarPos("Lightness Upper threshold", "Control Panel"), 0, 0);
    //Writing the updated threshold values on status bar 
    std::ostringstream sst;
    if (use_white)
        sst << "hl:" << 180 - cv::getTrackbarPos("Hue Lower threshold", "") << " hu:" << cv::getTrackbarPos("Hue Upper threshold", "") << " sl:" << 255 - cv::getTrackbarPos("Saturation Lower threshold", "") << "vl:" << 255 - cv::getTrackbarPos("Value Lower threshold", "") << " lu:" << 255 - cv::getTrackbarPos("Lightness Upper threshold", "");
    if (!use_white)
        sst << "hl:" << 180 - cv::getTrackbarPos("Hue Lower threshold", "") << " hu:" << cv::getTrackbarPos("Hue Upper threshold", "") << " sl:" << 255 - cv::getTrackbarPos("Saturation Lower threshold", "") << "vl:" << 255 - cv::getTrackbarPos("Value Lower threshold", "");
    std::string view = std::string(sst.str());
    cv::displayStatusBar("Multiple Outputs", view, 1000);//function to show data on status bar
    return;
}
//Function to change parameter for trackbar change for single laser ranging enabled
void ImageProcessing::myhandler_single(int red)
{
    myhandler(red);
    flag = true;
    return;
}
//Callback function for the white use button 
void ImageProcessing::on_button(int state, void *ptr)
{
    //This pointer is passed to change parameters
    ImageProcessing *c = (ImageProcessing *)(ptr);
    c->myhandlerbutton(state);
    return;
}
//Funtion to change parameter according to use_white button
void ImageProcessing::myhandlerbutton(int state)
{
    if (state == 1)
        use_white = true;
    else
        use_white = false;
    
    return;
}

ImageProcessing::ImageProcessing() //Constructor definition The values are preset in this constructor.
{
     //Threshold values preset for red color identification.
    thresh_low_0 = cv::Scalar(0, 140, 180, 0), thresh_high_0 = cv::Scalar(16, 255, 255, 0),
    thresh_low_180 = cv::Scalar(160, 160, 180, 0), thresh_high_180 = cv::Scalar(180, 255, 255, 0);
    thresh_white = cv::Scalar(0, 215, 0, 0);
    use_white = false; white_use_value = 0;
    //Region of interest preset for 30 percentage from center in height and full width.
    x = 50; y = 50; width = 100; height = 30;
    element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3), cv::Point(0, 0)); //Structuring element for dilation and erosion
}
