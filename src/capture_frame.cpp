
//Including the essential header files
#include <opencv2/opencv.hpp> 
#include <stdio.h>
#include <string.h>
#include "capture_frame.h"
#include "logger.h"

//capture the image into image varible
void CaptureFrame::capture_image(std::string filename,std::string image_window_name)
    {
        //Tries to load image.
        image = cv::imread(filename,1);
        if ( !image.data )
        {
            logger.log_error("No image data loaded");
            std::cout<<"No image data found for "<<filename<<"\n";// no input image found.exiting.
            exit(0);
        }
        window_name = image_window_name;
        return;
    }

//capture the video file into VideoCapture variable
void CaptureFrame::capture_video(std::string filename,std::string video_window_name)
    {
        //opens and read video 
        cap.open(filename);
        if(!cap.isOpened())  // check if we succeeded
        {
            logger.log_error("No video data loaded");
            printf("Video is not opened..:(");//The video couldn't be opened. exiting.
            exit(0);
        }
        window_name = video_window_name;
    }
//when camera number is given 
void CaptureFrame::capture_video(int camera,std::string video_window_name)
    {
        //opens camera and start reading
        std::cout<<"got here";
        cap.open(camera);
        if(!cap.isOpened())  // check if we succeeded
        {
            logger.log_error("No video data loaded");
            printf("Video is not opened..:(");//The video couldn't be opened. exiting.
            exit(0);
        }
        window_name = video_window_name;
        std::cout<<"test";
    }

//load image into already existing object of CaptureFrame class
void CaptureFrame::reload_image(cv::Mat image_input,std::string str)
    {
        if ( !image_input.data )
        {
            logger.log_error("No image data loaded");
            std::cout<<"No image data found for loading for "<<str<<"\n";// no input image found
            exit(0);
        }
        //Assignes new value to image and window name.
        image = image_input;
        window_name = str;
    }

void CaptureFrame::reload_video(cv::VideoCapture video_input,std::string str)
    {
        cap = video_input;
        window_name = str;
    }

//retrieve the image stored in object
cv::Mat CaptureFrame::retrieve_image()
    {
        //returns image file 
        return image;
    }

//retrieve the video file stored in object
cv::VideoCapture CaptureFrame::retrieve_video()
    {  
        //returns the VideoCapture file.
        return cap;
    }

//extracts frame from the contained video file and store it in image variable within the same object
void CaptureFrame::frame_extraction()
    {  
        //extracting the current frame to the image file.
        cap>>image;
        if ( !image.data )
        {
            logger.log_error("No image data found to extract");
            std::cout<<"No image data found for "<<window_name<<"\n";// no input image found
            throw(1);
        }//After this function call the current frame is saved in the image file of the same object.
        
        return;
    }

//Clear the values and release the memory allocated to every varibales
void CaptureFrame::clear()
    {
        //Clears all the varibles
        image.release();
        window_name.clear();
        cap.release();
        std::cout<<"Capture Frame cleared"<<"\n";
        return;
    }

//Constructer definitions
CaptureFrame::CaptureFrame(cv::Mat input,std::string window)
    {
        //Assignes the parameters
        image = input;
        window_name = window;
    }
CaptureFrame::CaptureFrame()
{}