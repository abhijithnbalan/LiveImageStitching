
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
    const float inlier_threshold = 2.5f; // Distance threshold to identify inliers
    const float nn_match_ratio = 0.8f;   // Nearest neighbor matching ratio
    
    cv::Mat img1,img2;
    image1 = resize_image(image1,20);
    image2 = resize_image(image2,20);
    // viewer.multiple_view_interrupted(image1,image2);


    // cv::cvtColor(image1.retrieve_image(),img1,CV_BGR2GRAY);
    // cv::cvtColor(image2.retrieve_image(),img2,CV_BGR2GRAY);
    img1 = image1.retrieve_image();
    img2 = image2.retrieve_image();
    std::vector<cv::KeyPoint> kpts1, kpts2;
    cv::Mat desc1, desc2;
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();
    akaze->detectAndCompute(img1, cv::noArray(), kpts1, desc1);
    akaze->detectAndCompute(img2, cv::noArray(), kpts2, desc2);
    std::cout<<"Akaze keypoints done\n";

    // cv::Mat imgkey1,imgkey2;
    // cv::drawKeypoints(img1,kpts1,imgkey1);
    // cv::drawKeypoints(img2,kpts2,imgkey2);
    // cv::imshow("key1",imgkey1);
    // cv::imshow("key2",imgkey2);    
    // cv::waitKey(20);

    // cv::drawMatches( img1, kpts1, img2, kpts2,
    //            good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
    //            std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

    
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector< std::vector<cv::DMatch> > nn_matches;
    matcher.knnMatch(desc1, desc2, nn_matches, 2);
    std::vector<cv::KeyPoint> inliers1, inliers2;
    std::vector<cv::Point2d> matched1, matched2;
    std::vector<cv::DMatch> good_matches;
    for(size_t i = 0; i < nn_matches.size(); i++) {
        cv::DMatch first = nn_matches[i][0];
        float dist1 = nn_matches[i][0].distance;
        float dist2 = nn_matches[i][1].distance;
        if(dist1 < nn_match_ratio * dist2) {
            matched1.push_back(kpts1[first.queryIdx].pt);
            matched2.push_back(kpts2[first.trainIdx].pt);
        }
    }

    cv::Mat homography = cv::findHomography( matched1, matched2, CV_RANSAC);

    // std::cout<<homography;

    for(unsigned i = 0; i < matched1.size(); i++) {
        cv::Mat col = cv::Mat::ones(3, 1, CV_64F);
        col.at<double>(0) = matched1[i].x;
        col.at<double>(1) = matched1[i].y;
        col = homography * col;  
        col /= col.at<double>(2);
        double dist = sqrt( pow(col.at<double>(0) - matched2[i].x, 2) +
                            pow(col.at<double>(1) - matched2[i].y, 2));
        if(dist < inlier_threshold) {
            int new_i = static_cast<int>(inliers1.size());
            inliers1.push_back(cv::KeyPoint(matched1[i],1.0f));
            inliers2.push_back(cv::KeyPoint(matched2[i],1.0f));
            good_matches.push_back(cv::DMatch(new_i, new_i, 0));
        }
    }
    cv::Mat res;
    cv::drawMatches(img1, inliers1, img2, inliers2, good_matches, res);
    cv::imshow("respng", res);
    cv::waitKey(0);
    if(good_matches.size()<4)
    {
        std::cout<<"Complete homography cannot be found with available frames\n";
        return;
    }

    cv::Mat big_pic(img1.cols+img2.cols,img1.rows+img2.rows,CV_8UC3);
    cv::Mat outimg1,outimg2;
    cv::Mat mask1(img1.size(), CV_8UC1, cv::Scalar::all(255));
    cv::Mat mask2(img2.size(), CV_8UC1, cv::Scalar::all(255));
    warpPerspective(img1, outimg1, homography,big_pic.size());
    warpPerspective(img2, outimg2, homography,big_pic.size());
    warpPerspective(mask1, mask1, homography,big_pic.size());
    warpPerspective(mask2, mask2, homography,big_pic.size());

    

    // cv::detail::MultiBandBlender blender(false, 5);
    // blender.feed(outimg1, mask1, cv::Point2f (0,0));
    // blender.feed(outimg2, mask2, cv::Point2f (0,0));
    //prepare resulting size of image
    std::cout<<"blah blah passed\n";
    // blender.prepare(cv::Rect(0, 0, big_pic.size().width, big_pic.size().height));
    cv::Mat result_s, result_mask;
    // blender.blend(result_s, result_mask);
    // img2.copyTo(big_pic);
    // big_pic = big_pic + outimg1;
    // outimg2.copyTo(big_pic);
    // cv::imshow("the big picture",result_s);
    cv::imshow("warped ima1ge",outimg1);
    cv::imshow("warped ima2ge",outimg2);
    // // cv::imshow("warped image",img1);
    cv::waitKey(0);
    return;

}
void ImageMosaic::Opencv_Stitcher(CaptureFrame image1, CaptureFrame image2)
{
    ViewFrame viewer;
    Logger logger;
    std::vector<cv::Mat> vImg;
    cv::Mat rImg;
    // image1 = resize_image(image1, 20);
    // image2 = resize_image(image2, 20);
    viewer.multiple_view_interrupted(image1,image2);
    vImg.push_back(image1.retrieve_image());
    vImg.push_back(image2.retrieve_image());

    cv::Stitcher stitcher = cv::Stitcher::createDefault();

    cv::Stitcher::Status status = stitcher.stitch(vImg, rImg);

    if (cv::Stitcher::OK == status)
    {

        mosaic_image.reload_image(rImg,"Mosaic");
        success_stitch = true;
        cv::imshow("image",rImg);
        
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

void ImageMosaic::native_stitcher()
{
    ViewFrame viewer;
    Logger logger;

    cv::Stitcher stitcher = cv::Stitcher::createDefault();
    // cv::Mat temp_mosaic;
    cv::Stitcher::Status status = stitcher.stitch(image_vector, mosaic);

    if (cv::Stitcher::OK == status)
    {
        mosaic_image.reload_image(mosaic,"Mosaic");
        viewer.single_view_interrupted(mosaic_image);
    } 
    else
    {
        logger.log_warn("Images could not be stitched.");
        cv::waitKey(30);
        return;
    }

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
void ImageMosaic::ORB_feature_match(CaptureFrame image1,CaptureFrame image2)
{


    cv::Ptr<cv::ORB> orb = cv::ORB::create();
    current_image = image1.retrieve_image();
    next_image = image2.retrieve_image();
    orb->detectAndCompute(current_image, cv::noArray(), keypoints1, description1);
    orb->detectAndCompute(next_image, cv::noArray(), keypoints2, description2);
    return;
}


void ImageMosaic::view_keypoints()
{    
    cv::Mat imgkey1,imgkey2;
    cv::drawKeypoints(current_image,keypoints1,imgkey1);
    cv::drawKeypoints(next_image,keypoints2,imgkey2);
    cv::imshow("key1",imgkey1);
    cv::imshow("key2",imgkey2);
    cv::waitKey(0);
    return;

} 

void ImageMosaic::find_homography()
{
    homography = cv::findHomography( matched1, matched2, CV_RANSAC);
    std::cout<<homography<<"\n";
    return;
    
}
    
    
void ImageMosaic::good_match_selection()
{
    for(unsigned i = 0; i < matched1.size(); i++) {
        cv::Mat col = cv::Mat::ones(3, 1, CV_64F);
        col.at<double>(0) = matched1[i].x;
        col.at<double>(1) = matched1[i].y;
        col = homography * col;  
        col /= col.at<double>(2);
        double dist = sqrt( pow(col.at<double>(0) - matched2[i].x, 2) +
                            pow(col.at<double>(1) - matched2[i].y, 2));
        if(dist < inlier_threshold) {
            int new_i = static_cast<int>(inliers1.size());
            inliers1.push_back(cv::KeyPoint(matched1[i],1.0f));
            inliers2.push_back(cv::KeyPoint(matched2[i],1.0f));
            good_matched1.push_back(matched1[i]);
            good_matched2.push_back(matched2[i]);
            good_matches.push_back(cv::DMatch(new_i, new_i, 0));
        }
    }
    cv::Mat res;
    cv::drawMatches(current_image, inliers1, next_image, inliers2, good_matches, res);
    cv::imshow("Good Matches",res);
    cv::waitKey(0);
    return;
}

void ImageMosaic::find_actual_homography()
{
    homography = cv::findHomography( good_matched1, good_matched2, CV_RANSAC);
    std::cout<<homography<<"\n";
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
    cv::Mat big_pic(current_image.cols+next_image.cols,next_image.rows,CV_8UC3);
    cv::Mat mask1(current_image.size(), CV_8UC1, cv::Scalar::all(255));
    // cv::Mat big_pic_mask(big_pic.size(), CV_8UC1, cv::Scalar::all(255));
    warpPerspective(current_image, warped_image, homography,big_pic.size());
    // warpPerspective(img2, outimg2, homography,big_pic.size());
    warpPerspective(mask1, warped_mask, homography,big_pic.size());
    // warpPerspective(mask2, mask2, homography,big_pic.size());
    return;
}


void ImageMosaic::BF_matcher()
{
    cv::BFMatcher matcher(cv::NORM_HAMMING);
    std::vector< std::vector<cv::DMatch> > nn_matches;
    matcher.knnMatch(description1, description2, nn_matches, 2);
    for(size_t i = 0; i < nn_matches.size(); i++) 
    {
        cv::DMatch first = nn_matches[i][0];
        float dist1 = nn_matches[i][0].distance;
        float dist2 = nn_matches[i][1].distance;
        if(dist1 < nn_match_ratio * dist2) 
        {
            matched1.push_back(keypoints1[first.queryIdx].pt);
            matched2.push_back(keypoints2[first.trainIdx].pt);
        }
    }
    return;
}

void ImageMosaic::image_blender()
{
    next_image.convertTo(next_image, CV_16S);
    warped_image.convertTo(warped_image, CV_16S);
    cv::detail::FeatherBlender  blender(0.5f); //sharpness

    blender.prepare(cv::Rect(0,0,std::max(next_image.cols,warped_image.cols),std::max(next_image.rows,warped_image.rows)));
    cv::Mat mask2(next_image.size(), CV_8UC1, cv::Scalar::all(255));
    blender.feed(warped_image, warped_mask, cv::Point2f (0,0));
    blender.feed(next_image, mask2, cv::Point2f (0,0));
    cv::Mat result_s, result_mask;
    blender.blend(result_s, result_mask);
    result_s.convertTo(mosaic, CV_8U);
    cv::imshow("result",mosaic);
    cv::waitKey(0);
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
    std::cout<<"ORB and homo\n";
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

void ImageMosaic::live_mosaicing()
    {
        return;
    }
