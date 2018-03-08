

#include "algorithm.h"
#include "logger.h"
#include "capture_frame.h"
#include <opencv2/opencv.hpp>

CaptureFrame Algorithm::CLAHE_dehaze(CaptureFrame object) //CLAHE based basic dehazing algorithm
{
    cv::Mat segmented;
    segmented = CLAHE_dehaze(object.retrieve_image());
    CaptureFrame output(segmented, "Dehazed image");
    return output;
}
cv::Mat Algorithm::CLAHE_dehaze(cv::Mat object) //CLAHE based basic dehazing algorithm
{
    cv::Mat image_hsv;
    cvtColor(object, image_hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    split(image_hsv, channels);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(CLAHE_clip_limit);
    clahe->apply(channels[2], channels[2]);
    clahe->apply(channels[1], channels[1]);
    channels[2] = channels[2] * 0.85;
    merge(channels, image_hsv);
    cv::Mat dehazed;
    cvtColor(image_hsv, dehazed, cv::COLOR_HSV2BGR);
    // GaussianBlur(dehazed, dehazed, cv::Size(3, 3), 2, 2);
    return dehazed;
}
CaptureFrame Algorithm::hist_equalize(CaptureFrame object) //CLAHE based basic dehazing algorithm
{
    cv::Mat image_hsv;
    cvtColor(object.retrieve_image(), image_hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    split(image_hsv, channels);
    cv::equalizeHist(channels[1], channels[1]);
    channels[2] = channels[2] * 0.85;
    merge(channels, image_hsv);
    cv::Mat dehazed;
    cvtColor(image_hsv, dehazed, cv::COLOR_HSV2BGR);
    // GaussianBlur(dehazed, dehazed, cv::Size(3, 3), 2, 2);
    CaptureFrame output(dehazed, "Dehazed image");
    return output;
}

void Algorithm::set_CLAHE_clip_limit(int clip_limit)//function which set the clip limit in CLAHE dehazing
{
    CLAHE_clip_limit = clip_limit;
}
Algorithm::Algorithm()
{
    inlier_threshold = 2.5f; 
    nn_match_ratio = 0.8f; 
    CLAHE_clip_limit = 2;
}

//Akaze feature points identification
void Algorithm::AKAZE_feature_points(CaptureFrame image1, CaptureFrame image2)
{
    
    inlier_threshold = 2.5f; // Distance threshold to identify inliers
    
    //Input images for identifying the keypoints
    current_image = image1.retrieve_image().clone();
    previous_image = image2.retrieve_image().clone();

    //Creating akaze object
    cv::Ptr<cv::AKAZE> akaze = cv::AKAZE::create();

    //Detecting keypoints in the images
    akaze->detectAndCompute(current_image, cv::noArray(), keypoints_current_image, description_current_image);
    akaze->detectAndCompute(previous_image, cv::noArray(), keypoints_previous_image, description_previous_image);
    
    //logging
    logger.log_info("Akaze feature point idenfication");
    return;

}

//Orb feature points identification
void Algorithm::ORB_feature_points(CaptureFrame image1,CaptureFrame image2)
{
    //Creating ORB object
    cv::Ptr<cv::ORB> orb = cv::ORB::create();

    //Input images
    current_image = image1.retrieve_image();
    previous_image = image2.retrieve_image();

    //Detecting feature points through ORB
    orb->detectAndCompute(current_image, cv::noArray(), keypoints_current_image, description_current_image);
    orb->detectAndCompute(previous_image, cv::noArray(), keypoints_previous_image, description_previous_image);
    
    //logging
    logger.log_info("ORB feature point idenfication");
    return;
}

//Matches the images in which keypoints are identified already
void Algorithm::BF_matcher()
{  
    //Clearing previous data
    matched_current_image.clear();
    matched_previous_image.clear();

    //Creating BruteForce matcher object
    cv::BFMatcher matcher(cv::NORM_HAMMING);

    //Matching
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

    //logging
    logger.log_info("Brute Force matching");
    return;
}

