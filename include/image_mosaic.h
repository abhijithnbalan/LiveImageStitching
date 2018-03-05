#ifndef _image_mosaic_h
#define _image_mosaic_h

#include <opencv2/opencv.hpp> 
#include "image_processing.h"
#include "capture_frame.h"
#include "view_frame.h"
#include "logger.h"

class ImageMosaic : public ImageProcessing
{
    private:
        ViewFrame viewer;
        std::vector<cv::Mat> image_vector;
        std::vector<cv::KeyPoint> keypoints_previous_image, keypoints_current_image, inliers_current_image, inliers_previous_image;
        cv::Mat description_current_image, description_previous_image, homography_matrix;
        std::vector<cv::DMatch> good_matches;
        std::vector<cv::Point2d> matched_current_image, matched_previous_image,good_matched_current_image,good_matched_previous_image;
        cv::Mat current_image,previous_image,original_mask,warped_image, warped_mask;
        float inlier_threshold; // Distance threshold to identify inliers
        float nn_match_ratio; 
        bool success_stitch;
        Logger logger;
        
    protected:

    public:
        cv::Mat warp_offset;
        cv::Mat mosaic;
        CaptureFrame mosaic_image;
        int total_images;
        void AKAZE_feature_match(CaptureFrame image1,CaptureFrame image2);
        void Opencv_Stitcher(CaptureFrame image1, CaptureFrame image2);
        void Opencv_Stitcher();
        void native_stitcher();
        void BF_matcher();
        void find_homography();
        void find_actual_homography();
        void warp_image();
        void live_mosaicing(CaptureFrame vid);
        void good_match_selection();
        void image_vector_maker(int argc ,char **argv);
        void image_vector_maker(CaptureFrame vid);
        void view_keypoints();
        void video_mosaic(CaptureFrame input_video);
        void image_stream_recorder(CaptureFrame video,int frame_rate);
        void ORB_feature_match(CaptureFrame image1, CaptureFrame image2);
        void image_blender();
        void display_image_vector();
        int number_of_matches(CaptureFrame image1,CaptureFrame image2);
        int number_of_matches(cv::Mat image1,cv::Mat image2);
        ImageMosaic();

};

#endif