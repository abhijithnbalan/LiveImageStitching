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
        cv::Mat homography_matrix,prev_homography;
        std::vector<cv::DMatch> good_matches;
        std::vector<cv::Point2d> good_matched_current_image,good_matched_previous_image;
        cv::Mat original_mask,warped_image, warped_mask,blend_offset;
        bool success_stitch,mosaic_state;
        int previous_area;
        cv::Rect bounding_rect;
        cv::Mat big_pic;
        bool blend_once;
        int intermediate;
        Logger logger;//Can be used everywhere inside the class
        
    protected:

    public:
        bool mosaic_trigger,use_dehaze;
        int image_count;
        bool reset_mosaic,stop_mosaic;
        cv::Mat warp_offset;//used for stitching in all direction
        cv::Mat mosaic;
        CaptureFrame mosaic_image,mosai_image_stable;//the output variable
        int total_images;

        
        //Shows found out keypoints on the corresponding image
        void view_keypoints();
        //finding homography transformation matrix from match points and good matches
        void find_homography();
        void find_actual_homography();
        //filter good matches from the total matches
        void good_match_selection();
        //Shows the matches with corresponding picture
        void view_matches();
        //Warping images according to the homography matrix
        void warp_image();
        void warp_image_live();
        //blending the two images to get final single image : feather blending is used
        void image_blender();
        void image_blender_live();
        CaptureFrame crop_live();
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