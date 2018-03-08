#ifndef _algorithm_h
#define _algorithm_h

#include "capture_frame.h"


class Algorithm
{
  private:
  Logger logger;

  protected:

  public:

  std::vector<cv::Point2d> matched_current_image, matched_previous_image;
  std::vector<cv::KeyPoint> keypoints_previous_image,//keypoints variables
  keypoints_current_image, inliers_current_image, inliers_previous_image;
  cv::Mat description_current_image, description_previous_image;
  cv::Mat current_image,previous_image;
  float nn_match_ratio; 
    int CLAHE_clip_limit;

    
    
    float inlier_threshold; // Distance threshold to identify inliers

    //Image Enhancing

     //CLAHE based basic dehazing algorithm
    CaptureFrame CLAHE_dehaze(CaptureFrame input_image);
     //CLAHE based basic dehazing algorithm
    cv::Mat CLAHE_dehaze(cv::Mat input_image);
    // histogram equilization algorithm
    CaptureFrame hist_equalize(CaptureFrame input_image);
    //support function for CLAHE dehazing
    void set_CLAHE_clip_limit(int clip_limit);

    //Image Mosaicing

    //Akaze keypoint creation with discription
    void AKAZE_feature_points(CaptureFrame image1,CaptureFrame image2);
    //ORB keypoint creation with description
    void ORB_feature_points(CaptureFrame image1, CaptureFrame image2);
    //Brute Force matcher : creates matches
    void BF_matcher();

    Algorithm();//The constructor
};

#endif