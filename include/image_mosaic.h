#ifndef _image_mosaic_h
#define _image_mosaic_h

#include <opencv2/opencv.hpp> 
#include "image_processing.h"
#include "capture_frame.h"
#include "view_frame.h"

class ImageMosaic : public ImageProcessing
{
    private:
        ViewFrame viewer;
        std::vector<cv::Mat> image_vector;
        std::vector<cv::KeyPoint> keypoints1, keypoints2, inliers1, inliers2;
        cv::Mat description1, description2, homography;
        std::vector<cv::DMatch> good_matches;
        std::vector<cv::Point2d> matched1, matched2,good_matched1,good_matched2;
        cv::Mat current_image,next_image, warped_image, warped_mask;
        float inlier_threshold; // Distance threshold to identify inliers
        float nn_match_ratio; 

    protected:

    public:
        cv::Mat mosaic;
        CaptureFrame mosaic_image;
        int total_images;
        void AKAZE_feature_match(CaptureFrame image1,CaptureFrame image2);
        void Opencv_Stitcher(CaptureFrame image1, CaptureFrame image2);
        void native_stitcher();
        void BF_matcher();
        void find_homography();
        void find_actual_homography();
        void warp_image();
        void good_match_selection();
    
        void view_keypoints();
        void video_mosaic(CaptureFrame input_video);
        void image_stream_recorder(CaptureFrame video,int frame_rate);
        void ORB_feature_match(CaptureFrame image1, CaptureFrame image2);
        void image_blender();
        ImageMosaic();

};

#endif