
//The necessary header files are included.
#include "view_frame.h"
#include "capture_frame.h"
#include <stdio.h>
#include <string.h>
#include <opencv2/opencv.hpp>


void ViewFrame::single_view_interrupted(CaptureFrame object) //Shows a single output and wait for userkey to continue
{
    if (!object.retrieve_image().data)
    {
        if (!object.retrieve_video().isOpened())
        {
            printf("nothing to show.\n");//No data included in CaptureFrame object
            return;
        }
        else
        {
            cv::Mat temporary;
            for (;;)
            {
                //Video file showing
                object.retrieve_video() >> temporary;//frame extraction
                cv::imshow(object.window_name, temporary);
                if (!temporary.data)
                {
                    return;//end of video
                }
                if (cv::waitKey(0) >= 0) //Waits for a key press from user to continue.
                {
                    //destroyWindow that was just created
                    cv::destroyWindow(object.window_name);
                    break;
                }
            }
            return;
        }
    }
    else
    {
        //showing image
        cv::imshow(object.window_name, object.retrieve_image());
        if (cv::waitKey(0) >= 0) //Waits for a key press from user to continue
        {
            //destroyWindow that was just created
            cv::destroyWindow(object.window_name);
            return;
        }
    }
}

void ViewFrame::single_view_interrupted(CaptureFrame object, int percent) //Overloaded function which also resizes the output
{
    cv::Mat temporary;
    resize(object.retrieve_image(), temporary, cv::Size(object.retrieve_image().cols * percent / 100, object.retrieve_image().rows * percent / 100));
    CaptureFrame output(temporary, object.window_name );
    single_view_interrupted(output);
    return;
}

// The following single output functions are uninterrupted by user input and are used in loops.

void ViewFrame::single_view_uninterrupted(CaptureFrame object) //Shows output and continues. Used inside loops.
{
    if (!object.retrieve_image().data)
    {
        if (!object.retrieve_video().isOpened())
        {
            printf("nothing to show.\n");//no data in the CaptureFrame object
            cv::destroyAllWindows();
            exit(0);
        }
        else
        {
            cv::Mat temporary;
            for (;;)
            {   //show the video 
                object.retrieve_video() >> temporary;
                cv::imshow(object.window_name, temporary);
                if(cv::waitKey(1)>=0)return;
                
            }
            return;
        }
    }
    else
    {   //showing image
        cv::imshow(object.window_name, object.retrieve_image());
        return;
    }
}

void ViewFrame::single_view_uninterrupted(CaptureFrame object, int percent) //Overloaded function which also resizes the output
{
    cv::Mat temporary;
    resize(object.retrieve_image(), temporary, cv::Size(object.retrieve_image().cols * percent / 100, object.retrieve_image().rows * percent / 100));
    CaptureFrame output(temporary, object.window_name );
    single_view_uninterrupted(output);
    return;
}

//Multiple outputs
void ViewFrame::multiple_view_interrupted(CaptureFrame object1, CaptureFrame object2) //Shows two images side by side and waits for user input.
{
    CaptureFrame full_image = join_image_horizontal(object1,object2,1);
    single_view_interrupted(full_image,75);
    return;
}

void ViewFrame::multiple_view_interrupted(CaptureFrame object1, CaptureFrame object2, int percent) //Overloaded function which also resizes the output
{
    cv::Mat temporary1, temporary2;
    resize(object1.retrieve_image(), temporary1, cv::Size(object1.retrieve_image().cols * percent / 100, object1.retrieve_image().rows * percent / 100));
    resize(object2.retrieve_image(), temporary2, cv::Size(object2.retrieve_image().cols * percent / 100, object2.retrieve_image().rows * percent / 100));
    CaptureFrame output1(temporary1, object1.window_name +" Resized");
    CaptureFrame output2(temporary2, object2.window_name +" Resized");
    ViewFrame::multiple_view_interrupted(output1, output2);
    return;
}

void ViewFrame::multiple_view_interrupted(CaptureFrame object1, CaptureFrame object2, CaptureFrame object3) //Overloaded function to shows three images
{                                                                                      //Converting all single channel images to 3 channel images.
    CaptureFrame horizontal = join_image_horizontal(object1,object2,1);
    CaptureFrame full_image = join_image_vertical(horizontal,object3,0);
    single_view_interrupted(full_image,75);
    return;
}
void ViewFrame::multiple_view_interrupted(CaptureFrame object1, CaptureFrame object2, CaptureFrame object3, int percent) //Overloaded function which also resizes the output
{
    cv::Mat temporary1, temporary2, temporary3;
    resize(object1.retrieve_image(), temporary1, cv::Size(object1.retrieve_image().cols * percent / 100, object1.retrieve_image().rows * percent / 100));
    resize(object2.retrieve_image(), temporary2, cv::Size(object2.retrieve_image().cols * percent / 100, object2.retrieve_image().rows * percent / 100));
    resize(object3.retrieve_image(), temporary3, cv::Size(object3.retrieve_image().cols * percent / 100, object3.retrieve_image().rows * percent / 100));
    CaptureFrame output1(temporary1, object1.window_name +" Resized");
    CaptureFrame output2(temporary2, object2.window_name +" Resized");
    CaptureFrame output3(temporary3, object3.window_name +" Resized");
    ViewFrame::multiple_view_interrupted(output1, output2, output3);
    return;
}
void ViewFrame::multiple_view_interrupted(CaptureFrame object1, CaptureFrame object2, CaptureFrame object3, CaptureFrame object4) //Overloaded function to show four outputs.
{                                                                                                            //Converting all single channel images to 3 channel image.
    CaptureFrame horizontal1 = join_image_horizontal(object1,object2,1);                                     //Converting all single channel images to 3 channel image.
    CaptureFrame horizontal2 = join_image_horizontal(object3,object4,1);                                     //Converting all single channel images to 3 channel image.
    CaptureFrame full_image = join_image_vertical(horizontal1,horizontal2,0);
    single_view_interrupted(full_image,75);
    return;
}
void ViewFrame::multiple_view_interrupted(CaptureFrame object1, CaptureFrame object2, CaptureFrame object3, CaptureFrame object4, int percent) //Overloaded function which also resizes the output
{
    cv::Mat temporary1, temporary2, temporary3, temporary4;
    resize(object1.retrieve_image(), temporary1, cv::Size(object1.retrieve_image().cols * percent / 100, object1.retrieve_image().rows * percent / 100));
    resize(object2.retrieve_image(), temporary2, cv::Size(object2.retrieve_image().cols * percent / 100, object2.retrieve_image().rows * percent / 100));
    resize(object3.retrieve_image(), temporary3, cv::Size(object3.retrieve_image().cols * percent / 100, object3.retrieve_image().rows * percent / 100));
    resize(object4.retrieve_image(), temporary4, cv::Size(object4.retrieve_image().cols * percent / 100, object4.retrieve_image().rows * percent / 100));
    CaptureFrame output1(temporary1,object1.window_name );
    CaptureFrame output2(temporary2,object2.window_name );
    CaptureFrame output3(temporary3,object3.window_name );
    CaptureFrame output4(temporary4,object4.window_name );
    ViewFrame::multiple_view_interrupted(output1, output2, output3, output4);
    return;
}

//The following 3 functions are the overloaded functions of the above modified for usage in loops.

void ViewFrame::multiple_view_uninterrupted(CaptureFrame object1, CaptureFrame object2) //2 Images
{
    CaptureFrame full_image = join_image_horizontal(object1,object2,1);
    single_view_uninterrupted(full_image,75);
    return;
}
void ViewFrame::multiple_view_uninterrupted(CaptureFrame object1, CaptureFrame object2, int percent) //Overloaded function which also resizes the output
{
    cv::Mat temporary1, temporary2;
    resize(object1.retrieve_image(), temporary1, cv::Size(object1.retrieve_image().cols * percent / 100, object1.retrieve_image().rows * percent / 100));
    resize(object2.retrieve_image(), temporary2, cv::Size(object2.retrieve_image().cols * percent / 100, object2.retrieve_image().rows * percent / 100));
    CaptureFrame output1(temporary1,object1.window_name );
    CaptureFrame output2(temporary2,object2.window_name );
    ViewFrame::multiple_view_uninterrupted(output1, output2);
    return;
}
void ViewFrame::multiple_view_uninterrupted(CaptureFrame object1, CaptureFrame object2, CaptureFrame object3) //3 Images
{
    CaptureFrame horizontal = join_image_horizontal(object1,object2,1);
    CaptureFrame full_image = join_image_vertical(horizontal,object3,0);
    single_view_uninterrupted(full_image,75);
    return;
}
void ViewFrame::multiple_view_uninterrupted(CaptureFrame object1, CaptureFrame object2, CaptureFrame object3, int percent) //Overloaded function which also resizes the output
{
    cv::Mat temporary1, temporary2, temporary3;
    resize(object1.retrieve_image(), temporary1, cv::Size(object1.retrieve_image().cols * percent / 100, object1.retrieve_image().rows * percent / 100));
    resize(object2.retrieve_image(), temporary2, cv::Size(object2.retrieve_image().cols * percent / 100, object2.retrieve_image().rows * percent / 100));
    resize(object3.retrieve_image(), temporary3, cv::Size(object3.retrieve_image().cols * percent / 100, object3.retrieve_image().rows * percent / 100));
    CaptureFrame output1(temporary1, object1.window_name );
    CaptureFrame output2(temporary2, object2.window_name );
    CaptureFrame output3(temporary3, object3.window_name );
    ViewFrame::multiple_view_uninterrupted(output1, output2, output3);
    return;
}
void ViewFrame::multiple_view_uninterrupted(CaptureFrame object1, CaptureFrame object2, CaptureFrame object3, CaptureFrame object4) //4 Images
{
    CaptureFrame horizontal1 = join_image_horizontal(object1,object2,1);
    CaptureFrame horizontal2 = join_image_horizontal(object3,object4,1);
    CaptureFrame full_image = join_image_vertical(horizontal1,horizontal2,0);
    single_view_uninterrupted(full_image,75);
    return;
}
void ViewFrame::multiple_view_uninterrupted(CaptureFrame object1, CaptureFrame object2, CaptureFrame object3, CaptureFrame object4, int percent) //Overloaded function which also resizes the output
{
    cv::Mat temporary1, temporary2, temporary3, temporary4;
    resize(object1.retrieve_image(), temporary1, cv::Size(object1.retrieve_image().cols * percent / 100, object1.retrieve_image().rows * percent / 100));
    resize(object2.retrieve_image(), temporary2, cv::Size(object2.retrieve_image().cols * percent / 100, object2.retrieve_image().rows * percent / 100));
    resize(object3.retrieve_image(), temporary3, cv::Size(object3.retrieve_image().cols * percent / 100, object3.retrieve_image().rows * percent / 100));
    resize(object4.retrieve_image(), temporary4, cv::Size(object4.retrieve_image().cols * percent / 100, object4.retrieve_image().rows * percent / 100));
    CaptureFrame output1(temporary1, object1.window_name );
    CaptureFrame output2(temporary2, object2.window_name );
    CaptureFrame output3(temporary3, object3.window_name );
    CaptureFrame output4(temporary4, object4.window_name );
    ViewFrame::multiple_view_uninterrupted(output1, output2, output3, output4);
    return;
}
// Overloaded Functions to overlay data onto the image
CaptureFrame ViewFrame::add_overlay(CaptureFrame object,int x, int y, std::string data)//For String when point is known
{
    cv::Mat image = object.retrieve_image();

    putText(image,data, cvPoint(x,y), cv::FONT_HERSHEY_COMPLEX_SMALL, 1.2, cvScalar(100,50,250), 1, CV_AA);//Adding time into frame
            
    CaptureFrame output(image,"overlayed output");
    return output;
}
CaptureFrame ViewFrame::add_overlay_percent(CaptureFrame object,int x_percent, int y_percent, int data)//for integer on required position
{
    cv::Mat image = object.retrieve_image();
    int x = x_percent*image.cols/100;
    int y = y_percent*image.rows/100;

    std::ostringstream sst;
    sst << data;
    std::string s(sst.str());

    CaptureFrame output = add_overlay(object,x,y,s);
    return output;
}
//String with font size and thickness
CaptureFrame ViewFrame::add_overlay_percent(CaptureFrame object,int x_percent, int y_percent, std::string data,cv::Scalar color,float size,int thick)
{
    cv::Mat image = object.retrieve_image();
    int x = x_percent*image.cols/100;
    int y = y_percent*image.rows/100;

    std::ostringstream sst;
    sst << data;
    std::string s(sst.str());
    putText(image,data, cvPoint(x,y), cv::FONT_HERSHEY_COMPLEX_SMALL, double(size), color, thick, CV_AA);//Adding time into frame
      
    CaptureFrame output(image,"overlayed image");
    return output;
}
//String with font size and thickness for Image
cv::Mat ViewFrame::add_overlay_percent(cv::Mat input,int x_percent, int y_percent, std::string data,cv::Scalar color,float size,int thick)
{
    cv::Mat image = input;
    int x = x_percent*image.cols/100;
    int y = y_percent*image.rows/100;

    std::ostringstream sst;
    sst << data;
    std::string s(sst.str());
    putText(image,data, cvPoint(x,y), cv::FONT_HERSHEY_COMPLEX_SMALL, double(size), color, thick, CV_AA);//Adding time into frame
    return image;
}
CaptureFrame ViewFrame::add_overlay(CaptureFrame object,int x, int y, int data)//for integer when point is known
{
    cv::Mat image = object.retrieve_image();

    std::ostringstream sst;
    sst << data;
    std::string s(sst.str());
    
    CaptureFrame output = add_overlay(object,x,y,s);
    return output;
}
CaptureFrame ViewFrame::add_overlay(CaptureFrame object,cv::Rect box, cv::Mat data)//Overlay image through a known rectange
{
    cv::Mat image = object.retrieve_image();

    image(box) = image(box).clone() + data;
    CaptureFrame output(image,"overlayed image");
    return output;
}
CaptureFrame ViewFrame::add_overlay_percent(CaptureFrame object,int x_percent, int y_percent, float data)//for floating point numbers with required position
{
    cv::Mat image = object.retrieve_image();
    int x = x_percent*image.cols/100;
    int y = y_percent*image.rows/100;

    std::ostringstream sst;
    sst << data;
    std::string s(sst.str());

    CaptureFrame output = add_overlay(object,x,y,s);
    return output;
}
CaptureFrame ViewFrame::join_image_horizontal(CaptureFrame object1,CaptureFrame object2,int mode)
{
    cv::Mat image1 = object1.retrieve_image().clone();
    cv::Mat image2 = object2.retrieve_image().clone();

    if (image1.channels() != 3)
    {
        
        if (image1.channels() == 1)
        {
            cvtColor(image1, image1, CV_GRAY2BGR);
        }
        else
        {
            std::cout << "images have different number of channels" << "\n";
            return object1;
        }
    }
    if (image2.channels() != 3)
    {
        
        if (image2.channels() == 1)
        {
            cvtColor(image2, image2, CV_GRAY2BGR);
        }
        else
        {
            std::cout << "images have different number of channels" << "\n";
            return object2;
        }
    }
    int row_max = std::max(image1.rows,image2.rows);
    cv::Mat full_image = cv::Mat::zeros(row_max, image1.cols + image2.cols, CV_8UC3);
    cv::Rect sub_roi;
    sub_roi.x = 0;
    sub_roi.y = 0;
    sub_roi.height = image1.rows;
    sub_roi.width = image1.cols;
    image1.copyTo(full_image(sub_roi));
    if(mode == 1) full_image(sub_roi) = add_overlay_percent(full_image(sub_roi),15,10,object1.window_name,cv::Scalar(200,200,250),1.5,2);
    sub_roi.x = image1.cols;
    sub_roi.width = image2.cols;
    sub_roi.height = image2.rows;
    image2.copyTo(full_image(sub_roi));
    if(mode == 1) full_image(sub_roi) = add_overlay_percent(full_image(sub_roi),15,10,object2.window_name,cv::Scalar(200,200,250),1.5,2);
    // resize(full_image, full_image, cv::Size(full_image.cols * 0.5, full_image.rows * 0.5));
    CaptureFrame output(full_image,"Multiple Outputs");
    return output;
}

CaptureFrame ViewFrame::join_image_vertical(CaptureFrame object1,CaptureFrame object2,int mode)
{
    cv::Mat image1 = object1.retrieve_image().clone();
    cv::Mat image2 = object2.retrieve_image().clone();

    if (image1.channels() != 3)
    {
        
        if (image1.channels() == 1)
        {
            cvtColor(image1, image1, CV_GRAY2BGR);
        }
        else
        {
            std::cout << "images have different number of channels" << "\n";
            return object1;
        }
    }
    if (image2.channels() != 3)
    {
        
        if (image2.channels() == 1)
        {
            cvtColor(image2, image2, CV_GRAY2BGR);
        }
        else
        {
            std::cout << "images have different number of channels" << "\n";
            return object2;
        }
    }
    int col_max = std::max(image1.cols,image2.cols);
    cv::Mat full_image = cv::Mat::zeros(image1.rows + image2.rows,col_max, CV_8UC3);
    cv::Rect sub_roi;
    sub_roi.x = 0;
    sub_roi.y = 0;
    sub_roi.height = image1.rows;
    sub_roi.width = image1.cols;
    image1.copyTo(full_image(sub_roi));
    if(mode == 1) full_image(sub_roi) = add_overlay_percent(full_image(sub_roi),15,10,object1.window_name,cv::Scalar(200,200,250),1.5,2);
    sub_roi.y = image1.rows;
    sub_roi.width = image2.cols;
    sub_roi.height = image2.rows;
    image2.copyTo(full_image(sub_roi));
    if(mode == 1) full_image(sub_roi) = add_overlay_percent(full_image(sub_roi),15,10,object2.window_name,cv::Scalar(200,200,250),1.5,2);
    CaptureFrame output(full_image,"Multiple Outputs");
    return output;
}