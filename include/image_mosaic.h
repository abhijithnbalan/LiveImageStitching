#ifndef _image_mosaic_h
#define _image_mosaic_h

//User defined 
#include "image_processing.h"
#include "capture_frame.h"
#include "view_frame.h"
#include "logger.h"
//Standard libraries
#include <opencv2/opencv.hpp> 

class ImageMosaic : public ImageProcessing
{
    private:
        ViewFrame viewer;// can be used anywhere within the class to show output
        std::vector<cv::Mat> image_vector;//used for opencv stitcher
        std::vector<cv::KeyPoint> keypoints_previous_image,//keypoints variables
        keypoints_current_image, inliers_current_image, inliers_previous_image;
        cv::Mat description_current_image, description_previous_image, homography_matrix;
        std::vector<cv::DMatch> good_matches;
        std::vector<cv::Point2d> matched_current_image, matched_previous_image,good_matched_current_image,good_matched_previous_image;
        cv::Mat current_image,previous_image,original_mask,warped_image, warped_mask;
        float inlier_threshold; // Distance threshold to identify inliers
        float nn_match_ratio; 
        bool success_stitch;
        Logger logger;//Can be used everywhere inside the class
        
    protected:

    public:
        int image_count;
        bool reset_mosaic,stop_mosaic;
        cv::Mat warp_offset;//used for stitching in all direction
        cv::Mat mosaic;
        CaptureFrame mosaic_image;//the output variable
        int total_images;

        //Akaze keypoint creation with discription
        void AKAZE_feature_points(CaptureFrame image1,CaptureFrame image2);
        //ORB keypoint creation with description
        void ORB_feature_points(CaptureFrame image1, CaptureFrame image2);
        //Shows found out keypoints on the corresponding image
        void view_keypoints();
        //Brute Force matcher : creates matches
        void BF_matcher();
        //finding homography transformation matrix from match points and good matches
        void find_homography();
        void find_actual_homography();
        //filter good matches from the total matches
        void good_match_selection();
        //Shows the matches with corresponding picture
        void view_matches();
        //Warping images according to the homography matrix
        void warp_image();
        //blending the two images to get final single image : feather blending is used
        void image_blender();
        //Image vector maker : makes image vecotr from video or set of images passed as arguments
        void image_vector_maker(int argc ,char **argv);
        void image_stream_recorder(CaptureFrame video,int frame_rate);
        //Shows the loded image vecotr
        void display_image_vector();
        //Finds out the number of good matches between the two images
        int number_of_matches(CaptureFrame image1,CaptureFrame image2);
        int number_of_matches(cv::Mat image1,cv::Mat image2);
        //OpenCV stitcher : uses image vecor to stitch
        void Opencv_Stitcher(CaptureFrame image1, CaptureFrame image2);
        void Opencv_Stitcher();
        //video mosaic using OpenCV stitcher
        void video_mosaic(CaptureFrame input_video);
        //Live mosaicing from video or camera 
        void live_mosaicing_video(CaptureFrame vid);
        void live_mosaicing_camera(CaptureFrame vid);
        //Construcor
        ImageMosaic();

};

#endif