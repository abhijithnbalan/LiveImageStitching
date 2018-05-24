
//Including necessary library files

//User defined
#include "image_mosaic.h"
#include "capture_frame.h"
#include "view_frame.h"
#include "logger.h"

//OpenCV specific
#include <opencv2/opencv.hpp>
#include "opencv2/imgproc/imgproc.hpp"
#include <opencv2/features2d.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/stitching.hpp>

//Standard libraries
#include <stdio.h>
#include <vector>
#include <iostream>
#include <iomanip>
#include <unistd.h>
#include <string.h>

//Creates image vector from a video with specified frame interval.
void ImageMosaic::image_stream_recorder(CaptureFrame video, int frame_rate)
{
    logger.debug_mode = dev_mode;
    //Extracting frames at specific interval
    for (int i = 0; i < frame_rate; i++)
    {
        video.frame_extraction();
    }

    // when extracted frame has data
    while (video.retrieve_image().data)
    {
        //Creating image vector.
        image_vector.push_back(video.retrieve_image());
        for (int i = 0; i < frame_rate; i++)
        {
            try
            {
                video.frame_extraction();
            }
            catch (int)
            {
                // logging
                logger.log_warn("end of video reached");
                return;
            }
        }
    }
    //logging
    logger.log_info("image stream recording completed");
}

//Image mosaic class constructor
ImageMosaic::ImageMosaic()
{
    //Initialising variables
    total_images = 0;
    image_count = 0;
    success_stitch = false;
    stop_mosaic = false;
    mosaic_state = false;
    reset_mosaic = false;
    use_dehaze = false;
    mosaic_trigger = false;
    previous_area = 0;
    intermediate = 1;
    camera_frame_delay = 5000;
    video_frame_skip = 5;

    dev_mode = true;
    roi_x = 50;
    roi_y = 50;
    roi_height = 100;
    roi_width = 100;
}

//show keypoints on image
void ImageMosaic::view_keypoints()
{
    //creating images to show keypoints
    cv::Mat keypoints_current_view, keypoints_previous_view;

    //drawing keypoints
    cv::drawKeypoints(algo.current_image, algo.keypoints_current_image, keypoints_current_view);
    cv::drawKeypoints(algo.previous_image, algo.keypoints_previous_image, keypoints_previous_view);

    //showing keypoints and waiting for user input to continue
    cv::imshow("Key Points in the current frame", keypoints_current_view);
    cv::imshow("Key Ponts in the previous frmae", keypoints_previous_view);
    //logging
    logger.log_info("Keypoints shown");

    cv::waitKey(0);
    return;
}

//Finding homography matrix from the matches
void ImageMosaic::find_homography()
{
    //Using RANSAC to find homography transformation matrix
    try
    {
        homography_matrix = cv::findHomography(algo.matched_current_image, algo.matched_previous_image, CV_RANSAC);
    }
    catch (...)
    {
        logger.log_error("Homography matrix could not be calculated.");
        throw 1;
    }

    // std::cout<<homography_matrix<<"\n";

    //logging
    logger.log_info("Homography matrix calculated");

    return;
}

//Filter out good matches among the matched keypoints with respect to the distance
void ImageMosaic::good_match_selection()
{
    //Clearing previous data from vectors
    good_matches.clear();
    good_matched_current_image.clear();
    good_matched_previous_image.clear();
    algo.inliers_current_image.clear();
    algo.inliers_previous_image.clear();

    //Filtering good matches
    for (unsigned i = 0; i < algo.matched_current_image.size(); i++)
    {
        cv::Mat col = cv::Mat::ones(3, 1, CV_64F);
        col.at<double>(0) = algo.matched_current_image[i].x;
        col.at<double>(1) = algo.matched_current_image[i].y;
        col = homography_matrix * col;
        col /= col.at<double>(2);
        double dist = sqrt(pow(col.at<double>(0) - algo.matched_previous_image[i].x, 2) +
                           pow(col.at<double>(1) - algo.matched_previous_image[i].y, 2));
        if (dist < algo.inlier_threshold)
        {
            int new_i = static_cast<int>(algo.inliers_current_image.size());
            algo.inliers_current_image.push_back(cv::KeyPoint(algo.matched_current_image[i], 1.0f));
            algo.inliers_previous_image.push_back(cv::KeyPoint(algo.matched_previous_image[i], 1.0f));
            good_matched_current_image.push_back(algo.matched_current_image[i]);
            good_matched_previous_image.push_back(algo.matched_previous_image[i]);
            good_matches.push_back(cv::DMatch(new_i, new_i, 0));
        }
    }

    //logging
    logger.log_info("Good matches filtered");

    return;
}

//view the filtered matches
void ImageMosaic::view_matches()
{
    cv::Mat image_matches;
    cv::drawMatches(algo.current_image, algo.inliers_current_image, algo.previous_image, algo.inliers_previous_image, good_matches, image_matches);
    cv::imshow("Good Matches", image_matches);

    //logging
    logger.log_info("Good matches shown");
    cv::waitKey(0);
    return;
}

//Recalculating homography with filtered good matches
void ImageMosaic::find_actual_homography()
{
    //using good matches instead of normal matches
    try
    {
        homography_matrix = cv::findHomography(good_matched_current_image, good_matched_previous_image, CV_RANSAC);
    }
    catch (...)
    {
        logger.log_error("Actual Homography matrix could not be calculated");
    }
    // std::cout<<homography_matrix<<"\n";

    //logger
    logger.log_info("Actual Homography matrix calculated");

    return;
}

//Warp the current image according to the homography found out
void ImageMosaic::warp_image()
{
    //Initializing big picture to accomodate the warped image and blending which will happen in later stage
    cv::Mat big_pic;
    big_pic.release();
    big_pic = cv::Mat::zeros(3 * algo.previous_image.rows, 3 * algo.previous_image.cols, CV_8UC3);
    //Masks are needed for blending and also need to be warped like original image
    cv::Mat mask1(algo.current_image.size(), CV_8UC1, cv::Scalar::all(255));
    cv::Mat mask2(algo.previous_image.size(), CV_8UC1, cv::Scalar::all(255));

    //Offset is multiplied with homogrophy matrix to place the image in the center so that it can be stitched in any direction.
    warp_offset = (cv::Mat_<double>(3, 3) << 1, 0, algo.current_image.cols, 0, 1, algo.current_image.rows, 0, 0, 1);

    cv::Mat effective_homography_matrix = warp_offset * homography_matrix;
    // std::cout<<effective_homography_matrix<<"\n";

    //Warping the images according to homography + translation
    warpPerspective(algo.current_image, warped_image, effective_homography_matrix, big_pic.size());
    warpPerspective(algo.previous_image, algo.previous_image, warp_offset, big_pic.size());
    warpPerspective(mask1, warped_mask, effective_homography_matrix, big_pic.size());
    warpPerspective(mask2, original_mask, warp_offset, big_pic.size());

    //logging
    logger.log_info("warping successful");
    return;
}
void ImageMosaic::warp_image_live()
{
    if (!big_pic.data)
    {
        big_pic = cv::Mat::zeros(3 * algo.current_image.rows, 3 * algo.current_image.cols, CV_8UC3);
    }

    //Masks are needed for blending and also need to be warped like original image
    cv::Mat mask1(algo.current_image.size(), CV_8UC1, cv::Scalar::all(255));
    cv::Mat mask2(algo.previous_image.size(), CV_8UC1, cv::Scalar::all(255));

    //Offset is multiplied with homogrophy matrix to place the image in the center so that it can be stitched in any direction.
    if (!warp_offset.data)
    {
        warp_offset = (cv::Mat_<double>(3, 3) << 1, 0, algo.current_image.cols, 0, 1, algo.current_image.rows, 0, 0, 1);
    }
    // std::cout<<prev_homography<<"\n";
    // std::cout<<homography_matrix<<"\n";

    effective_homography_matrix.release();
    effective_homography_matrix = warp_offset * prev_homography.clone() * homography_matrix.clone();
    // std::cout<<effective_homography_matrix<<"\n";
    // blend_offset = (cv::Mat_<double>(3,3) << 1, 0, -homography_matrix.at<int>(0,2), 0, 1,-homography_matrix.at<int>(1,2), 0, 0, 1);
    //Warping the images according to homography + translation

    warpPerspective(algo.current_image, warped_image, effective_homography_matrix, big_pic.size());
    warpPerspective(algo.previous_image, algo.previous_image, blend_offset, big_pic.size());
    warpPerspective(mask1, warped_mask, effective_homography_matrix, big_pic.size());
    warpPerspective(mask2, original_mask, blend_offset, big_pic.size());
    // viewer.single_view_uninterrupted()
    if (blend_once)
    {
        blend_offset = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    }
    blend_once = false;
    //logging
    logger.log_info("warping successful");
    return;
}

//Blending the two images together
void ImageMosaic::image_blender()
{
    //Converting into 16S for blending
    algo.previous_image.convertTo(algo.previous_image, CV_16S);
    warped_image.convertTo(warped_image, CV_16S);

    //Creating feather blend for blending images with specified sharpness

    cv::detail::FeatherBlender blender(0.09f); //sharpness

    //Blender preparing
    blender.prepare(cv::Rect(0, 0, std::max(algo.previous_image.cols, warped_image.cols), std::max(algo.previous_image.rows, warped_image.rows)));

    //Feeding the images with points to blend
    blender.feed(algo.previous_image, original_mask, cv::Point2f(0, 0));
    blender.feed(warped_image, warped_mask, cv::Point2f(0, 0));

    //Blending the fed images
    cv::Mat result_s, result_mask;
    blender.blend(result_s, result_mask);
    result_s.convertTo(mosaic, CV_8UC3);
    cv::Mat gray_mosaic = cv::Mat::zeros(mosaic.size(), CV_8UC1);
    cv::cvtColor(mosaic, gray_mosaic, cv::COLOR_BGR2GRAY);
    logger.log_info("Blending successful");
    // cv::imshow("result",mosaic);
    // cv::waitKey(0);

    //Find the contour with largest area to crop the imges out from the big picture
    int largest_area = 0;
    int largest_contour_index = 0;
    // cv::Rect bounding_rect;
    std::vector<std::vector<cv::Point>> contours; // Vector for storing contour
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(gray_mosaic, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    // iterate through each contour.
    for (int i = 0; i < contours.size(); i++)
    {
        //  Find the area of contour
        double a = contourArea(contours[i], false);
        if (a > largest_area)
        {
            largest_area = a;
            // Store the index of largest contour
            largest_contour_index = i;
            // Find the bounding rectangle for biggest contour
            bounding_rect = boundingRect(contours[i]);
        }
    }
    if (largest_area < (previous_area * 1.5) || previous_area == 0)
    {
        previous_area = largest_area;
    }
    else
    {
        logger.log_warn("Bad mapping identified. Negleting frame");
        return;
    }
    //Cropping the image
    cv::Mat cropped_image = mosaic(bounding_rect).clone();
    // Loading the image to public CaptureFrame object
    mosaic_image.reload_image(cropped_image, "mosaic");

    //logging
    logger.log_info("Blending and cropping successful");
    return;
}

void ImageMosaic::image_blender_live()
{
    // //Converting into 16S for blending
    algo.previous_image.convertTo(algo.previous_image, CV_16S);
    warped_image.convertTo(warped_image, CV_16S);

    //Creating feather blend for blending images with specified sharpness

    cv::detail::FeatherBlender blender(0.05f); //sharpness
    // cv::detail::MultiBandBlender  blender(false,1);

    //Blender preparing
    blender.prepare(cv::Rect(0, 0, std::max(algo.previous_image.cols, warped_image.cols), std::max(algo.previous_image.rows, warped_image.rows)));

    //Feeding the images with points to blend
    blender.feed(algo.previous_image, original_mask, cv::Point2f(0, 0));
    blender.feed(warped_image, warped_mask, cv::Point2f(0, 0));

    //Blending the fed images
    cv::Mat result_s, result_mask;
    blender.blend(result_s, result_mask);

    result_s.convertTo(mosaic, CV_8UC3);

    // cv::cvtColor(mosaic,mosaic,CV_BGR2HSV);

    // cv::cvtColor(mosaic,mosaic,CV_HSV2BGR);

    cv::Mat gray_mosaic = cv::Mat::zeros(mosaic.size(), CV_8UC1);
    cv::cvtColor(mosaic, gray_mosaic, cv::COLOR_BGR2GRAY);
    // cv::imshow("result",mosaic);
    // cv::waitKey(0);

    //Find the contour with largest area to crop the imges out from the big picture
    int largest_area = 0;
    int largest_contour_index = 0;
    bounding_rect = cv::Rect(0, 0, 0, 0);
    std::vector<std::vector<cv::Point>> contours; // Vector for storing contour
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(gray_mosaic, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    // iterate through each contour.
    for (int i = 0; i < contours.size(); i++)
    {
        //  Find the area of contour
        double a = contourArea(contours[i], false);
        if (a > largest_area)
        {
            largest_area = a;
            // Store the index of largest contour
            largest_contour_index = i;
            // Find the bounding rectangle for biggest contour
            bounding_rect = boundingRect(contours[i]);
        }
    }
    if (largest_area < (previous_area * 1.5) || previous_area == 0)
    {
        previous_area = largest_area;
    }
    else
    {
        logger.log_warn("Bad mapping identified. Negleting frame");
        return;
    }

    int new_row = big_pic.rows;
    int new_col = big_pic.cols;
    if (bounding_rect.x < big_pic.cols / 8)
    {
        logger.log_debug("big picture expanded to left");
        warp_offset.at<double>(0, 2) = warp_offset.at<double>(0, 2) + big_pic.cols / 8;
        blend_offset.at<double>(0, 2) = blend_offset.at<double>(0, 2) + big_pic.cols / 8;
        blend_once = true;
        new_col = new_col + big_pic.cols / 8;
    }
    if ((bounding_rect.x + bounding_rect.width) > (7 * big_pic.cols / 8))
    {
        logger.log_debug("big picture expanded to right");
        new_col = new_col + big_pic.cols / 8;
    }
    if (bounding_rect.y < big_pic.rows / 8)
    {
        logger.log_debug("big picture expanded to up");
        warp_offset.at<double>(1, 2) = warp_offset.at<double>(1, 2) + big_pic.rows / 8;
        blend_offset.at<double>(1, 2) = blend_offset.at<double>(1, 2) + big_pic.rows / 8;
        blend_once = true;
        new_row = new_row + big_pic.rows / 8;
    }
    if ((bounding_rect.y + bounding_rect.height) > (7 * big_pic.rows / 8))
    {
        logger.log_debug("big picture expanded to down");
        new_row = new_row + big_pic.rows / 8;
    }

    big_pic = cv::Mat::zeros(new_row, new_col, CV_8UC3);

    //Cropping the image
    // cv::Mat cropped_image = mosaic(bounding_rect).clone();
    cv::Mat brightness_compensation = cv::Mat::ones(mosaic.size(), CV_8UC1);
    std::vector<cv::Mat> channels;
    cv::split(mosaic, channels);
    cv::add(channels[0], brightness_compensation, channels[0]);
    cv::add(channels[1], brightness_compensation, channels[1]);
    cv::add(channels[2], brightness_compensation, channels[2]);
    cv::merge(channels, mosaic);
    // Loading the image to public CaptureFrame object
    mosaic_image.reload_image(mosaic, "mosaic");
    mosaic_image_stable = mosaic_image;

    //logging
    logger.log_info("Blending successful");
    return;
}

CaptureFrame ImageMosaic::crop_live()
{
    cv::Mat full_image = mosaic_image_stable.retrieve_image().clone();
    cv::Mat cropped_image = full_image(bounding_rect).clone();
    CaptureFrame output(cropped_image, "cropped mosaic");
    return output;
}

//finding the number of matches between two images
int ImageMosaic::number_of_matches(CaptureFrame image1, CaptureFrame image2)
{
    //feature points
    algo.AKAZE_feature_points(image1, image2);
    //Brute Force matching
    algo.BF_matcher();
    //homography calcualtion
    find_homography();
    //Good match filtering
    good_match_selection();

    //logging
    logger.log_info("Number of Good matches");

    //returning the number of elements in good matches
    return good_matches.size();
}

//number of matches when inputs are cv::Mat images
int ImageMosaic::number_of_matches(cv::Mat image1, cv::Mat image2)
{
    //Converting to CaptureFrame
    CaptureFrame img1(image1, "first image");
    CaptureFrame img2(image2, "second image");
    // Function call;
    return number_of_matches(img1, img2);
}
//Image vector making with all images passed as arguments
void ImageMosaic::image_vector_maker(int argc, char **argv)
{
    logger.debug_mode = debug_mode;
    //Taking each images passed as argument one by one and creating the vector
    cv::Mat temp;
    for (int i = 1; i < argc; i++)
    {
        temp = cv::imread(argv[i], 1);
        image_vector.push_back(temp);
    }
    return;
}

//To display the recorded image vector for inspection
void ImageMosaic::display_image_vector()
{
    //Creating ViewFrame object
    ViewFrame viewer;

    //Showing images one by one
    CaptureFrame temp;
    for (int i = 0; i < image_vector.size(); i++)
    {
        temp = CaptureFrame(image_vector[i], "image vector");
        if (dev_mode)
        {
            viewer.single_view_interrupted(temp, 70);
        }
    }
    return;
}

//Native stitcher of OpenCV. Need image vector
void ImageMosaic::Opencv_Stitcher(CaptureFrame image1, CaptureFrame image2)
{

    //Clearing image vector
    image_vector.clear();

    //Creating image vector with the two images
    image_vector.push_back(image1.retrieve_image());
    image_vector.push_back(image2.retrieve_image());

    //OpenCV stitcher with precreated image vector
    Opencv_Stitcher();
}

//Native stitcher by OpenCV with precreated image vector
void ImageMosaic::Opencv_Stitcher()
{
    logger.debug_mode = debug_mode;
    //Image output
    cv::Mat image_out;
    //Creating stitcher object
    cv::Stitcher stitcher = cv::Stitcher::createDefault();

    //Stitching : successful stitch implies status is OK
    cv::Stitcher::Status status = stitcher.stitch(image_vector, image_out);

    if (cv::Stitcher::OK == status)
    {
        //Loading image to public CaptureFrame object.
        mosaic_image.reload_image(image_out, "Mosaic");
        //stitcher status. Used for the further execution.
        success_stitch = true;

        //logging
        logger.log_info("OpenCV stitching successful");
    }
    else //Stitching unsuccessful
    {
        //logging
        logger.log_warn("OpenCV stitching failed");
        cv::waitKey(30);
        return;
    }

    cv::waitKey(30);
    return;
}

//live mosaic can be used to stitch from video or camera feed in realtime
void ImageMosaic::live_mosaicing_video(CaptureFrame vid)
{
    logger.debug_mode = debug_mode;
    set_roi(roi_x, roi_y, roi_height, roi_width);
    //live mosaicing is used to stitch imges in realtme
    //extracting frames
    try
    {
        vid.frame_extraction(video_frame_skip);
    }
    catch (...)
    {
        logger.log_warn("video done already");
        return;
    }

    ViewFrame viewer;
    blend_once = true;
    // current frame and previous frames will be used to stitch images
    CaptureFrame current_frame;
    CaptureFrame previous_frame;
    //Initialising as number of matches to zero
    int matches = 0;
    previous_frame = CaptureFrame(roi_selection(vid).retrieve_image(), "previous frame");
    if (use_dehaze)
        previous_frame = algo.CLAHE_dehaze(previous_frame);
    //Extracting frames and loading it in current and previous images
    try
    {
        vid.frame_extraction(video_frame_skip);
        current_frame = CaptureFrame(roi_selection(vid).retrieve_image(), "current frame");
    }
    catch (...)
    {
        logger.log_warn("video ended ");
        return;
    }

    //Initialising homographies used in live mosaic
    prev_homography = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    blend_offset = (cv::Mat_<double>(3, 3) << 1, 0, current_frame.retrieve_image().cols, 0, 1, current_frame.retrieve_image().rows, 0, 0, 1);

    // stitching in loop and number of images are counted
    for (image_count = 0;;)
    {
        current_frame.reload_image(roi_selection(vid).retrieve_image(), "latest frame");
        if (use_dehaze)
        {
            current_frame = algo.CLAHE_dehaze(current_frame);
        }

        if (dev_mode)
        {
            viewer.single_view_uninterrupted(roi_selection(vid), 60);
        }
        char c = (char)cv::waitKey(75);
        if (c == 116 || c == 84 || mosaic_trigger) //checking for 't' or 'T' to toggle the trigger
        {
            if (mosaic_trigger)
            {

                mosaic_trigger = !mosaic_trigger; //Toggling the current value in mosaic_trigger
            }
            if (!mosaic_image.retrieve_image().data)
            {
                if (use_dehaze)
                    previous_frame = algo.CLAHE_dehaze(roi_selection(vid));
                else
                    previous_frame.reload_image(roi_selection(vid).retrieve_image(), "previous frame");
            }
            try
            {
                vid.frame_extraction(video_frame_skip);
                // current_frame.reload_image(vid.retrieve_image().clone(), "current frame");
            }
            catch (...)
            {
                logger.log_warn("video ended ");
                return;
            }

            mosaic_state = !mosaic_state;
            logger.log_warn("User interruption. Toggling mosaic state..");

            continue;
        }
        else if (c == 114 || c == 82 || reset_mosaic) //Checking for 'r' or 'R' to reset the mosaic image.
        {
            if (reset_mosaic)
            {
                reset_mosaic = !reset_mosaic;
            }
            logger.log_warn("User interruption. resetting the image..");

            if (crop_live().retrieve_image().data)
            {
                std::string file_prename = intermediate_filename.substr(0,intermediate_filename.size()-4);
                std::string filename = std::to_string(intermediate);

                cv::imwrite(file_prename + "_" + filename + ".jpg", crop_live().retrieve_image().clone());
                intermediate++;
                logger.log_info("Intermediate Mosaic image " + filename + ".jpg" + " is saved to disk");
                mosaic_image.clear();
                // mosaic_image.reload_image(current_frame.retrieve_image().clone(),"mosaic image");
                previous_frame.clear();
                big_pic.release();
                warp_offset.release();
                warp_offset = (cv::Mat_<double>(3, 3) << 1, 0, current_frame.retrieve_image().cols / 2, 0, 1, current_frame.retrieve_image().rows / 2, 0, 0, 1);

                prev_homography.release();
                blend_offset.release();
                prev_homography = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
                // homography_matrix = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
                blend_offset = (cv::Mat_<double>(3, 3) << 1, 0, current_frame.retrieve_image().cols / 2, 0, 1, current_frame.retrieve_image().rows / 2, 0, 0, 1);
                blend_once = true;
                mosaic_image.reload_image(current_frame.retrieve_image().clone(), "mosaic");
                // mosaic_image.reload_image(cv::Mat::zeros(current_frame.retrieve_image().size(),CV_8UC3),"mosaic image");
                // viewer.single_view_uninterrupted(mosaic_image,25);
                previous_frame.reload_image(current_frame.retrieve_image().clone(), "previous frame");
                // algo.previous_image.release();
                // algo.current_image.release();
                homography_matrix.release();
            }

            // mosaic_image.clear();
            // previous_frame.clear();
            // big_pic.release();
            // prev_homography.release();
            // prev_homography = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
            // blend_offset = (cv::Mat_<double>(3, 3) << 1, 0, current_frame.retrieve_image().cols, 0, 1, current_frame.retrieve_image().rows, 0, 0, 1);
            // blend_once = true;
            // mosaic_image.reload_image(current_frame.retrieve_image().clone(), "mosaic");
            // // viewer.single_view_uninterrupted(mosaic_image,25);
            // previous_frame.reload_image(current_frame.retrieve_image().clone(), "previous frame");
            // homography_matrix.release();
            vid.frame_extraction(video_frame_skip / 2);
            image_count = 1;
            continue;
        }
        else if (c == 115 || c == 83 || stop_mosaic) //checking for 's' or 'S' to stop mosaicing
        {
            // stop_mosaic = !stop_mosaic;
            logger.log_warn("User interruption. Stopping mosaic..");
            cv::destroyAllWindows();
            break;
        }

        if (mosaic_state)
        {

            algo.AKAZE_feature_points(current_frame, previous_frame);
            algo.BF_matcher();
            try
            {
                find_homography();
            }
            catch (...)
            {
                logger.log_info("Couldn't Find Homography. Skipping frame");
                //start extracting frames for next iteration
                try
                {
                    vid.frame_extraction(video_frame_skip / 2);

                    // previous_frame.reload_image(current_frame.retrieve_image().clone(),"swap image");
                }
                catch (...)
                {
                    logger.log_warn("end of video reached");
                    break;
                }
                continue;
            }
            // logger.log_warn("good match");
            try
            {
                good_match_selection();
                matches = good_matches.size();
            }
            catch (...)
            {
                logger.log_info("Couldn't Find Homography. Skipping frame");
                //start extracting frames for next iteration
                try
                {
                    vid.frame_extraction(video_frame_skip / 2);
                }
                catch (...)
                {
                    logger.log_warn("end of video reached");
                    break;
                }
                continue;
            }

            //Stitch images only if more than 10 good matches are obtained. 4 is the theoratical minimum.
            if (matches > 10)
            {

                if (mosaic_image.retrieve_image().data)
                {
                    algo.previous_image.release();
                    algo.current_image.release();
                    algo.previous_image = mosaic_image.retrieve_image().clone();
                    algo.current_image = current_frame.retrieve_image().clone();
                }

                try
                {
                    find_actual_homography();
                }
                catch (...)
                {
                    logger.log_info("skipping frame");
                    continue;
                }
                // logger.log_warn("warp image");
                warp_image_live();
                image_blender_live();

                prev_homography = prev_homography * homography_matrix;
                homography_matrix.release();

                //Show live mosaic result
                image_count++;
                if (dev_mode)
                {
                    viewer.single_view_uninterrupted(mosaic_image, 20);
                }
                cv::waitKey(5);

                //start extracting frames for next iteration
                try
                {
                    vid.frame_extraction(video_frame_skip);

                    previous_frame.reload_image(current_frame.retrieve_image().clone(), "swap image");
                }
                catch (...)
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
                    vid.frame_extraction(video_frame_skip);
                }
                catch (...)
                {
                    logger.log_warn("end of video reached");
                    break;
                }
            }
        }
        else
        {
            if (dev_mode)
            {
                viewer.single_view_uninterrupted(roi_selection(vid));
            }
            try
            {
                vid.frame_extraction();
            }
            catch (...)
            {
                logger.log_warn("end of video reached");
                break;
            }
        }
    }
    logger.log_warn("Image mosaicing completed");
    previous_frame.clear();
    current_frame.clear();
    algo.keypoints_current_image.clear();
    algo.keypoints_previous_image.clear();
    good_matched_current_image.clear();
    good_matched_previous_image.clear();
    good_matches.clear();
    algo.matched_current_image.clear();
    algo.matched_previous_image.clear();

    std::cout << "\nImage count : " << image_count << "\n";

    return;
}
void ImageMosaic::live_mosaicing_camera(CaptureFrame vid)
{
    logger.debug_mode = debug_mode;
    //live mosaicing is used to stitch imges in realtime from camera
    set_roi(roi_x, roi_y, roi_height, roi_width);
    //extracting frames
    try
    {
        vid.frame_extraction();
    }
    catch (...)
    {
        logger.log_error("Camera stopped working. Exiting");
        return;
    }

    ViewFrame viewer;
    blend_once = true;
    // current frame and previous frames will be used to stitch images
    CaptureFrame current_frame;
    CaptureFrame previous_frame;
    //Initialising as number of matches to zero
    int matches = 0;
    previous_frame = CaptureFrame(roi_selection(vid).retrieve_image(), "previous frame");
    if (use_dehaze)
        previous_frame = algo.CLAHE_dehaze(previous_frame);
    try
    {
        vid.frame_extraction(video_frame_skip);
        current_frame = CaptureFrame(roi_selection(vid).retrieve_image(), "current frame");
    }
    catch (...)
    {
        logger.log_warn("video ended ");
        return;
    }

    //Initialising homographies used in live mosaic
    prev_homography = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    blend_offset = (cv::Mat_<double>(3, 3) << 1, 0, current_frame.retrieve_image().cols, 0, 1, current_frame.retrieve_image().rows, 0, 0, 1);

    // stitching in loop and number of images are counted
    for (image_count = 0;;)
    {
        current_frame.reload_image(roi_selection(vid).retrieve_image(), "latest frame");
        if (use_dehaze)
        {
            current_frame = algo.CLAHE_dehaze(current_frame);
        }

        if (dev_mode)
        {
            viewer.single_view_uninterrupted(roi_selection(vid), 60);
        }
        char c = (char)cv::waitKey(75);
        if (c == 116 || c == 84 || mosaic_trigger) //checking for 't' or 'T' to stop mosaicing
        {
            if (mosaic_trigger)
            {
                mosaic_trigger = !mosaic_trigger;
            }

            if (!mosaic_image.retrieve_image().data)
            {
                try
                {
                    vid.frame_extraction();
                }
                catch (...)
                {
                    logger.log_error("Camera stopped working");
                    return;
                }
                if (use_dehaze)
                    previous_frame = algo.CLAHE_dehaze(roi_selection(vid));
                else
                    previous_frame.reload_image(roi_selection(vid).retrieve_image(), "previous frame");
            }
            usleep(camera_frame_delay);

            try
            {
                vid.frame_extraction();
                // current_frame.reload_image(vid.retrieve_image().clone(), "current frame");
            }
            catch (...)
            {
                logger.log_error("Camera stopped working ");
                return;
            }

            // blend_once = true;

            mosaic_state = !mosaic_state;
            logger.log_warn("User interruption. toggling mosaic state..");

            continue;
        }
        else if (c == 114 || c == 82 || reset_mosaic) //Checking for 'r' or 'R' to reset the mosaic image.
        {
            if (reset_mosaic)
            {
                reset_mosaic = !reset_mosaic;
            }
            logger.log_warn("User interruption. resetting the image..");
            if (crop_live().retrieve_image().data)
            {
                std::string file_prename = intermediate_filename.substr(0,intermediate_filename.size()-4);
                
                std::string filename = std::to_string(intermediate);
                cv::imwrite(file_prename + filename + ".jpg", crop_live().retrieve_image().clone());
                intermediate++;
                logger.log_info("Intermediate Mosaic image " + filename + ".jpg" + " is saved to disk");
                // mosaic_image.clear();
                // mosaic_image.reload_image(current_frame.retrieve_image().clone(),"mosaic image");
                previous_frame.clear();
                big_pic.release();
                prev_homography.release();
                prev_homography = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
                blend_offset = (cv::Mat_<double>(3, 3) << 1, 0, current_frame.retrieve_image().cols, 0, 1, current_frame.retrieve_image().rows, 0, 0, 1);
                blend_once = true;
                // mosaic_image.reload_image(current_frame.retrieve_image().clone(), "mosaic");
                mosaic_image.reload_image(cv::Mat::zeros(current_frame.retrieve_image().size(), CV_8UC3), "mosaic image");
                // viewer.single_view_uninterrupted(mosaic_image,25);
                previous_frame.reload_image(current_frame.retrieve_image().clone(), "previous frame");
                homography_matrix.release();
            }
            usleep(camera_frame_delay / 2);
            vid.frame_extraction();
            image_count = 1;
            continue;
        }
        else if (c == 115 || c == 83 || stop_mosaic) //checking for 's' or 'S' to stop mosaicing
        {
            logger.log_warn("User interruption. stopping..");
            cv::destroyAllWindows();
            break;
        }
        if (mosaic_state)
        {

            algo.AKAZE_feature_points(current_frame, previous_frame);
            algo.BF_matcher();
            try
            {
                find_homography();
            }
            catch (...)
            {
                logger.log_warn("Couldn't Find Homography. Skipping frame");
                try
                {
                    usleep(camera_frame_delay / 2);
                    vid.frame_extraction();

                    // previous_frame.reload_image(current_frame.retrieve_image().clone(),"swap image");
                }
                catch (...)
                {
                    logger.log_warn("end of video reached");
                    break;
                }
                continue;
            }

            try
            {
                good_match_selection();
                matches = good_matches.size();
            }
            catch (...)
            {
                logger.log_info("Couldn't Find Homography. Skipping frame");
                //start extracting frames for next iteration
                try
                {
                    usleep(camera_frame_delay / 2);
                    vid.frame_extraction();
                }
                catch (...)
                {
                    logger.log_warn("end of video reached");
                    break;
                }
                continue;
            }

            //Stitch images only if more than 10 good matches are obtained. 4 is the theoratical minimum.
            if (matches > 10)
            {
                if (mosaic_image.retrieve_image().data)
                {
                    algo.previous_image.release();
                    algo.current_image.release();
                    algo.previous_image = mosaic_image.retrieve_image().clone();
                    algo.current_image = current_frame.retrieve_image().clone();
                }

                try
                {
                    find_actual_homography();
                }
                catch (...)
                {
                    logger.log_info("skipping frame");
                    continue;
                }
                try
                {
                    warp_image_live();
                    image_blender_live();
                }
                catch (...)
                {
                    logger.log_warn("Image couldn't be mapped. Skipping frame");
                    continue;
                }

                prev_homography = prev_homography * homography_matrix;
                homography_matrix.release();

                //Show live mosaic result
                image_count++;
                if (dev_mode)
                {
                    viewer.single_view_uninterrupted(mosaic_image, 75);
                }
                cv::waitKey(5);

                //start extracting frames for next iteration
                try
                {
                    usleep(camera_frame_delay);
                    //delay
                    vid.frame_extraction();

                    previous_frame.reload_image(current_frame.retrieve_image().clone(), "current frame as first image");
                }
                catch (...)
                {
                    logger.log_error("Camera stopped");
                    break;
                }
            }
            else
            {
                logger.log_error("The images cannot be stitched");
                try
                {
                    usleep(camera_frame_delay); // delay
                    vid.frame_extraction();
                }
                catch (...)
                {
                    logger.log_error("Camera stopped");
                    break;
                }
            }
        }
        else
        {
            if (dev_mode)
            {
                viewer.single_view_uninterrupted(vid);
            }
            try
            {
                usleep(200);
                vid.frame_extraction();
            }
            catch (...)
            {
                logger.log_warn("end of video reached");
                break;
            }
        }
    }
    logger.log_warn("Image mosaicing completed");
    previous_frame.clear();
    current_frame.clear();
    algo.keypoints_current_image.clear();
    algo.keypoints_previous_image.clear();
    good_matched_current_image.clear();
    good_matched_previous_image.clear();
    good_matches.clear();
    algo.matched_current_image.clear();
    algo.matched_previous_image.clear();

    std::cout << "\nImage count : " << image_count << "\n";
    usleep(20000);
    return;
}