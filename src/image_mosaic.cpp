
#include "image_mosaic.h"
#include "capture_frame.h"
#include "view_frame.h"
#include <opencv2/opencv.hpp> 
#include "opencv2/imgproc/imgproc.hpp"
#include <stdio.h>

#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/stitching.hpp>

#include <vector>
#include <iostream>
#include <iomanip>
#include "logger.h"
 

void ImageMosaic::AKAZE_feature_match(CaptureFrame image1, CaptureFrame image2)
{
    inlier_threshold = 2.5f; // Distance threshold to identify inliers
    
    current_image = image1.retrieve_image().clone();
    previous_image = image2.retrieve_image().clone();
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
    akaze->detectAndCompute(current_image, cv::noArray(), keypoints_current_image, description_current_image);
    akaze->detectAndCompute(previous_image, cv::noArray(), keypoints_previous_image, description_previous_image);

    return;

}
void ImageMosaic::ORB_feature_match(CaptureFrame image1,CaptureFrame image2)
{


    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    current_image = image1.retrieve_image();
    previous_image = image2.retrieve_image();
    orb->detectAndCompute(current_image, cv::noArray(), keypoints_current_image, description_current_image);
    orb->detectAndCompute(previous_image, cv::noArray(), keypoints_previous_image, description_previous_image);
    return;
}
void ImageMosaic::BF_matcher()
{  
    matched_current_image.clear();
    matched_previous_image.clear();
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector< std::vector<cv::DMatch> > nn_matches;
    matcher.knnMatch(description_current_image, description_previous_image, nn_matches, 2);
    for(size_t i = 0; i < nn_matches.size(); i++) 
    {
        cv::DMatch first = nn_matches[i][0];
        float dist1 = nn_matches[i][0].distance;
        float dist2 = nn_matches[i][1].distance;
        if(dist1 < nn_match_ratio * dist2) 
        {
            matched_current_image.push_back(keypoints_current_image[first.queryIdx].pt);
            matched_previous_image.push_back(keypoints_previous_image[first.trainIdx].pt);
        }
    }
    return;
}
void ImageMosaic::Opencv_Stitcher(CaptureFrame image1, CaptureFrame image2)
{
    ViewFrame viewer;
    Logger logger;
    std::vector<cv::Mat> vImg;
    cv::Mat rImg;
    vImg.push_back(image1.retrieve_image());
    vImg.push_back(image2.retrieve_image());

    cv::Stitcher stitcher = cv::Stitcher::createDefault();

    cv::Stitcher::Status status = stitcher.stitch(vImg, rImg);

    if (cv::Stitcher::OK == status)
    {

        mosaic_image.reload_image(rImg,"Mosaic");
        success_stitch = true;
        // cv::imshow("image",rImg);
        
    } 
    else
    {
        logger.log_warn("Images could not be stitched.");
        cv::waitKey(30);
        return;
    }

    cv::waitKey(30);
    return;
}
void ImageMosaic::Opencv_Stitcher()
{
    ViewFrame viewer;
    Logger logger;
    cv::Mat rImg;
    // image1 = resize_image(image1, 20);
    // image2 = resize_image(image2, 20);

    cv::Stitcher stitcher = cv::Stitcher::createDefault();

    cv::Stitcher::Status status = stitcher.stitch(image_vector, rImg);

    if (cv::Stitcher::OK == status)
    {
        mosaic_image.reload_image(rImg,"Mosaic");
        success_stitch = true;
    } 
    else
    {
        logger.log_warn("Images could not be stitched.");
        cv::waitKey(30);
        return;
    }

    cv::waitKey(30);
    return;
}
void ImageMosaic::image_stream_recorder(CaptureFrame video,int frame_rate)
{
    for(int i = 0; i < frame_rate; i++)
        {
            video.frame_extraction();
        }

    while(video.retrieve_image().data)
    {
        image_vector.push_back(video.retrieve_image());
        for(int i = 0; i < frame_rate; i++)
        {
            try
            {
                video.frame_extraction();
            }
            catch(int)
            {
                return;
            }
        }
    }
}
ImageMosaic::ImageMosaic()
{
    total_images = 0;
    inlier_threshold = 2.5f; // Distance threshold to identify inliers
    nn_match_ratio = 0.8f; 
    success_stitch = false;
}
void ImageMosaic::view_keypoints()
{    
    cv::Mat keypoints_current_view,keypoints_previous_view;
    cv::drawKeypoints(current_image,keypoints_current_image,keypoints_current_view);
    cv::drawKeypoints(previous_image,keypoints_previous_image,keypoints_previous_view);
    cv::imshow("Key Points in the current frame",keypoints_current_view);
    cv::imshow("Key Ponts in the previous frmae",keypoints_previous_view);
    cv::waitKey(0);
    return;

} 

void ImageMosaic::find_homography()
{
    homography_matrix = cv::findHomography( matched_current_image, matched_previous_image, CV_RANSAC);
    std::cout<<homography_matrix<<"\n";
    return;
    
}
    
void ImageMosaic::good_match_selection()
{
    good_matches.clear();
    good_matched_current_image.clear();
    good_matched_previous_image.clear();
    inliers_current_image.clear();
    inliers_previous_image.clear();
    for(unsigned i = 0; i < matched_current_image.size(); i++) {
        cv::Mat col = cv::Mat::ones(3, 1, CV_64F);
        col.at<double>(0) = matched_current_image[i].x;
        col.at<double>(1) = matched_current_image[i].y;
        col = homography_matrix * col;  
        col /= col.at<double>(2);
        double dist = sqrt( pow(col.at<double>(0) - matched_previous_image[i].x, 2) +
                            pow(col.at<double>(1) - matched_previous_image[i].y, 2));
        if(dist < inlier_threshold) {
            int new_i = static_cast<int>(inliers_current_image.size());
            inliers_current_image.push_back(cv::KeyPoint(matched_current_image[i],1.0f));
            inliers_previous_image.push_back(cv::KeyPoint(matched_previous_image[i],1.0f));
            good_matched_current_image.push_back(matched_current_image[i]);
            good_matched_previous_image.push_back(matched_previous_image[i]);
            good_matches.push_back(cv::DMatch(new_i, new_i, 0));
        }
    }
    logger.log_warn("last message");
    cv::Mat res;
    cv::drawMatches(current_image, inliers_current_image, previous_image, inliers_previous_image, good_matches, res);
    // cv::imshow("Good Matches",res);
    // cv::waitKey(0);
    return;
}

void ImageMosaic::find_actual_homography()
{
    homography_matrix = cv::findHomography( good_matched_current_image, good_matched_previous_image, CV_RANSAC);
    std::cout<<homography_matrix<<"\n";
    return;
    
}

void ImageMosaic::image_vector_maker(int argc, char **argv)
{
    cv::Mat temp;
    for (int i = 1; i < argc ; i++)
    {
        temp = cv::imread(argv[i],1);
        image_vector.push_back(temp);
    }
    return;
}

void ImageMosaic::image_vector_maker(CaptureFrame vid)
{
    cv::Mat temp;
    int matches = 0;
    vid.frame_extraction(10);
    logger.log_warn("image_vector making");
    image_vector.push_back(vid.retrieve_image());
    CaptureFrame first_img(vid.retrieve_image(),"first image");
    CaptureFrame second_img(vid.retrieve_image(),"second image");
    logger.log_warn("loop starts");
    for (int i = 1;; i++)
    {
        try 
            {
                vid.frame_extraction(50);
            }
            catch(...)
            {
                logger.log_warn("reached the end of  video. Exiting.. \n");
                break;  
            }
        second_img.reload_image(vid.retrieve_image(),"current frame");
        logger.log_warn("reload image");
        viewer.multiple_view_uninterrupted(first_img,second_img,70);
        matches = number_of_matches(first_img,second_img);
        logger.log_warn("number of matches");
        if(matches > 10)
        {
            image_vector.push_back(vid.retrieve_image().clone());
            try 
            {
                vid.frame_extraction(20);
                logger.log_warn("trying now");
            }
            catch(...)
            {
                logger.log_warn("reached the end of  video. Exiting.. \n");
                break;
            }
            first_img = second_img;
            logger.log_warn("more than 10 matches");
        }
        else 
        {
            try 
            {
                logger.log_warn("trying now");
                vid.frame_extraction(10);
                
            }
            catch(...)
            {
                logger.log_warn("reached the end of  video. Exiting.. \n");
                break;
            }
        }
    }
    std::cout<<"creation of image vector : done\n";
    return;
}


void ImageMosaic::warp_image()
{  
     cv::Mat big_pic;
    big_pic.release();
    big_pic = cv::Mat::zeros(3 * previous_image.cols,3 * previous_image.rows ,CV_8UC3);
    cv::Mat mask1(current_image.size(), CV_8UC1, cv::Scalar::all(255));
    cv::Mat mask2(previous_image.size(), CV_8UC1, cv::Scalar::all(255));
    // cv::Mat big_pic_mask(big_pic.size(), CV_8UC1, cv::Scalar::all(255));
    warp_offset = (cv::Mat_<double>(3,3) << 1, 0, current_image.cols/2, 0, 1,current_image.rows/2, 0, 0, 1);
    logger.log_warn("warping offset");
    std::cout<<warp_offset<<"\n";
    logger.log_warn("homography matrix premultiplied by offset");
    cv::Mat effective_homography_matrix = warp_offset * homography_matrix;
    std::cout<<effective_homography_matrix<<"\n";

    warpPerspective(current_image, warped_image, effective_homography_matrix,big_pic.size());
    warpPerspective(previous_image, previous_image, warp_offset,big_pic.size());
    warpPerspective(mask1, warped_mask,effective_homography_matrix,big_pic.size());
    warpPerspective(mask2, original_mask, warp_offset,big_pic.size());
    // cv::imshow("skfsidubfiu",warped_image);
    // cv::waitKey(0);
    return;
}

void ImageMosaic::image_blender()
{
    previous_image.convertTo(previous_image, CV_16S);
    warped_image.convertTo(warped_image, CV_16S);
    cv::detail::FeatherBlender  blender(0.5f); //sharpness

    blender.prepare(cv::Rect(0,0,std::max(previous_image.cols,warped_image.cols),std::max(previous_image.rows,warped_image.rows)));
    
    
    blender.feed(previous_image, original_mask, cv::Point2f (0,0));
    blender.feed(warped_image, warped_mask, cv::Point2f (0,0));
    
    cv::Mat result_s, result_mask;
    blender.blend(result_s, result_mask);
    result_s.convertTo(mosaic, CV_8UC3);
    cv::Mat gray_mosaic = cv::Mat::zeros(mosaic.size(),CV_8UC1);
    cv::cvtColor(mosaic,gray_mosaic,cv::COLOR_BGR2GRAY);
    logger.log_warn("blending fine");
    // cv::imshow("result",mosaic);
    // cv::waitKey(0);
    
    int largest_area=0;
    int largest_contour_index=0;
    cv::Rect bounding_rect;
    std::vector<std::vector<cv::Point>> contours; // Vector for storing contour
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours( gray_mosaic, contours, hierarchy,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE );
    // iterate through each contour.
    for( int i = 0; i< contours.size(); i++ )
    {
        //  Find the area of contour
        double a=contourArea( contours[i],false); 
        if(a>largest_area)
        {
            largest_area=a;
            // Store the index of largest contour
            largest_contour_index = i;    
            logger.log_warn("largest area contour found out");          
            // Find the bounding rectangle for biggest contour
            bounding_rect=boundingRect(contours[i]);
        }   
    }
    cv::Mat cropped_image = mosaic(bounding_rect).clone();
    // cv::imshow("result",cropped_image);
    mosaic_image.reload_image(cropped_image,"mosaic");
    // cv::waitKey(0);
    return;
}

int ImageMosaic::number_of_matches(CaptureFrame image1, CaptureFrame image2)
{
    ORB_feature_match(image1,image2);
    // logger.log_warn("ORB features : done");
    BF_matcher();
    // logger.log_warn("matching : done");
    find_homography();

    good_match_selection();

    return good_matches.size();
}

int ImageMosaic::number_of_matches(cv::Mat image1, cv::Mat image2)
{
    CaptureFrame img1(image1,"first image");
    CaptureFrame img2(image2,"second image");
    ORB_feature_match(img1,img2);
    
    BF_matcher();
    find_homography();
    logger.log_info("priliminary homography matrix");
    good_match_selection();
    return good_matches.size();
}
void ImageMosaic::display_image_vector()
{
    ViewFrame viewer;
    CaptureFrame temp;
    for (int i = 0; i < image_vector.size(); i++)
    {
        temp = CaptureFrame(image_vector[i],"image vector");
        viewer.single_view_interrupted(temp,70);
    }
    return;
}

void ImageMosaic::live_mosaicing(CaptureFrame vid)
{   
        try{vid.frame_extraction(5);}
        catch(...){logger.log_warn("video done already");return;}
        ViewFrame viewer;
        CaptureFrame current_frame;
        CaptureFrame previous_frame;
        int matches = 0;
        try{vid.frame_extraction(25);}
            catch(...){logger.log_warn("video ended ");return;}
            previous_frame = CaptureFrame(vid.retrieve_image(),"previous frame");
            try{vid.frame_extraction(25);}
            catch(...){logger.log_warn("video ended ");return;}
            current_frame = CaptureFrame(vid.retrieve_image(),"current frame");

        for(int image_count = 0; ; image_count++)
        {
            
            ORB_feature_match(current_frame,previous_frame);
            BF_matcher();
            find_homography();
            good_match_selection();
            matches = good_matches.size();
            if(matches > 10)
            {
                find_actual_homography();
                logger.log_warn("actual homo");
                warp_image();
                logger.log_warn("warping completed");
                image_blender();
                logger.log_warn("blended in");
                viewer.single_view_uninterrupted(mosaic_image,80);
                cv::waitKey(10);
                try
                {
                    vid.frame_extraction(25);
                    if(mosaic_image.retrieve_image().data)
                    {
                        previous_frame.reload_image(mosaic_image.retrieve_image().clone(),"mosaic as first image");
                    }
                    else previous_frame.reload_image(current_frame.retrieve_image().clone(),"mosaic as first image");
                    
                    current_frame.reload_image(vid.retrieve_image(),"current image");
                }
                catch(...)
                {
                    logger.log_warn("end of video reached");
                    break;
                }
            }
            else
            {
                logger.log_warn("The images cannot be stitched");
                try
                {
                    vid.frame_extraction(25);
                    // previous_frame = current_frame;
                    current_frame.reload_image(vid.retrieve_image(),"current image");
                }
                catch(...)
                {
                    logger.log_warn("end of video reached");
                    break;
                }
            }
            
        }
        logger.log_warn("Image mosaicing completed");
        cv::Mat output = mosaic_image.retrieve_image(); 
        cv::imwrite("Mosaic_image.jpg",output);
        return;
}
