
//Including necessary library files

//User defined 
#include "image_mosaic.h"
#include "capture_frame.h"
#include "view_frame.h"

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
#include "logger.h"
 
//Akaze feature points identification
void ImageMosaic::AKAZE_feature_points(CaptureFrame image1, CaptureFrame image2)
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
void ImageMosaic::ORB_feature_points(CaptureFrame image1,CaptureFrame image2)
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
void ImageMosaic::BF_matcher()
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

//Creates image vector from a video with specified frame interval.
void ImageMosaic::image_stream_recorder(CaptureFrame video,int frame_rate)
{
    //Extracting frames at specific interval
    for(int i = 0; i < frame_rate; i++)
        {
            video.frame_extraction();
        }

    // when extracted frame has data
    while(video.retrieve_image().data)
    {
        //Creating image vector.
        image_vector.push_back(video.retrieve_image());
        for(int i = 0; i < frame_rate; i++)
        {
            try
            {
                video.frame_extraction();
            }
            catch(int)
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
    inlier_threshold = 2.5f; 
    nn_match_ratio = 0.8f; 
    success_stitch = false;
}

//show keypoints on image
void ImageMosaic::view_keypoints()
{    
    //creating images to show keypoints
    cv::Mat keypoints_current_view,keypoints_previous_view;

    //drawing keypoints
    cv::drawKeypoints(current_image,keypoints_current_image,keypoints_current_view);
    cv::drawKeypoints(previous_image,keypoints_previous_image,keypoints_previous_view);

    //showing keypoints and waiting for user input to continue
    cv::imshow("Key Points in the current frame",keypoints_current_view);
    cv::imshow("Key Ponts in the previous frmae",keypoints_previous_view);
    //logging
    logger.log_info("Keypoints shown");

    cv::waitKey(0);
    return;

} 

//Finding homography matrix from the matches
void ImageMosaic::find_homography()
{
    //Using RANSAC to find homography transformation matrix
    homography_matrix = cv::findHomography( matched_current_image, matched_previous_image, CV_RANSAC);
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
    inliers_current_image.clear();
    inliers_previous_image.clear();

    //Filtering good matches
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

    //logging
    logger.log_info("Good matches filtered");
    
    return;
}

//view the filtered matches
void ImageMosaic::view_matches()
{
    cv::Mat image_matches;
    cv::drawMatches(current_image, inliers_current_image, previous_image, inliers_previous_image, good_matches, image_matches);
    cv::imshow("Good Matches",image_matches);

    //logging
    logger.log_info("Good matches shown");
    cv::waitKey(0);
    return;
}

//Recalculating homography with filtered good matches
void ImageMosaic::find_actual_homography()
{
    //using good matches instead of normal matches
    homography_matrix = cv::findHomography( good_matched_current_image, good_matched_previous_image, CV_RANSAC);
    // std::cout<<homography_matrix<<"\n";

    //logger
    logger.log_info("Homography matrix calculated");

    return;
    
}

//Warp the current image according to the homography found out
void ImageMosaic::warp_image()
{  
    //Initializing big picture to accomodate the warped image and blending which will happen in later stage
    cv::Mat big_pic;
    big_pic.release();
    big_pic = cv::Mat::zeros(3 * previous_image.cols,3 * previous_image.rows ,CV_8UC3);
    //Masks are needed for blending and also need to be warped like original image
    cv::Mat mask1(current_image.size(), CV_8UC1, cv::Scalar::all(255));
    cv::Mat mask2(previous_image.size(), CV_8UC1, cv::Scalar::all(255));
    
    //Offset is multiplied with homogrophy matrix to place the image in the center so that it can be stitched in any direction.
    warp_offset = (cv::Mat_<double>(3,3) << 1, 0, current_image.cols/2, 0, 1,current_image.rows/2, 0, 0, 1);
    
    cv::Mat effective_homography_matrix = warp_offset * homography_matrix;
    // std::cout<<effective_homography_matrix<<"\n";

    //Warping the images according to homography + translation
    warpPerspective(current_image, warped_image, effective_homography_matrix,big_pic.size());
    warpPerspective(previous_image, previous_image, warp_offset,big_pic.size());
    warpPerspective(mask1, warped_mask,effective_homography_matrix,big_pic.size());
    warpPerspective(mask2, original_mask, warp_offset,big_pic.size());

    //logging
    logger.log_info("warping successful");
    return;
}

//Blending the two images together
void ImageMosaic::image_blender()
{
    //Converting into 16S for blending
    previous_image.convertTo(previous_image, CV_16S);
    warped_image.convertTo(warped_image, CV_16S);

    //Creating feather blend for blending images with specified sharpness
    cv::detail::FeatherBlender  blender(0.5f); //sharpness

    //Blender preparing
    blender.prepare(cv::Rect(0,0,std::max(previous_image.cols,warped_image.cols),std::max(previous_image.rows,warped_image.rows)));
    
    //Feeding the images with points to blend
    blender.feed(previous_image, original_mask, cv::Point2f (0,0));
    blender.feed(warped_image, warped_mask, cv::Point2f (0,0));
    
    //Blending the fed images
    cv::Mat result_s, result_mask;
    blender.blend(result_s, result_mask);
    result_s.convertTo(mosaic, CV_8UC3);
    cv::Mat gray_mosaic = cv::Mat::zeros(mosaic.size(),CV_8UC1);
    cv::cvtColor(mosaic,gray_mosaic,cv::COLOR_BGR2GRAY);
    logger.log_info("Blending successful");
    // cv::imshow("result",mosaic);
    // cv::waitKey(0);
    
    //Find the contour with largest area to crop the imges out from the big picture
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
            // Find the bounding rectangle for biggest contour
            bounding_rect=boundingRect(contours[i]);
        }   
    }
    //Cropping the image
    cv::Mat cropped_image = mosaic(bounding_rect).clone();
    // Loading the image to public CaptureFrame object
    mosaic_image.reload_image(cropped_image,"mosaic");

    //logging
    logger.log_info("Blending and cropping successful");
    return;
}

//finding the number of matches between two images
int ImageMosaic::number_of_matches(CaptureFrame image1, CaptureFrame image2)
{
    //feature points
    ORB_feature_points(image1,image2);
    //Brute Force matching
    BF_matcher();
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
    CaptureFrame img1(image1,"first image");
    CaptureFrame img2(image2,"second image");
    // Function call;
    return number_of_matches(img1,img2);
}
//Image vector making with all images passed as arguments
void ImageMosaic::image_vector_maker(int argc, char **argv)
{
    //Taking each images passed as argument one by one and creating the vector
    cv::Mat temp;
    for (int i = 1; i < argc ; i++)
    {
        temp = cv::imread(argv[i],1);
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
        temp = CaptureFrame(image_vector[i],"image vector");
        viewer.single_view_interrupted(temp,70);
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
    //Image output
    cv::Mat image_out;
    //Creating stitcher object
    cv::Stitcher stitcher = cv::Stitcher::createDefault();

    //Stitching : successful stitch implies status is OK
    cv::Stitcher::Status status = stitcher.stitch(image_vector, image_out);

    if (cv::Stitcher::OK == status)
    {
        //Loading image to public CaptureFrame object.
        mosaic_image.reload_image(image_out,"Mosaic");
        //stitcher status. Used for the further execution.
        success_stitch = true;

        //logging
        logger.log_info("OpenCV stitching successful");
        
    } 
    else//Stitching unsuccessful
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
void ImageMosaic::live_mosaicing(CaptureFrame vid)
{
        //live mosaicing is used to stitch imges in realtme

        //extracting frames
        try{vid.frame_extraction(5);}
        catch(...){logger.log_warn("video done already");return;}

        ViewFrame viewer;

        // current frame and previous frames will be used to stitch images
        CaptureFrame current_frame;
        CaptureFrame previous_frame;
        //Initialising as number of matches to zero
        int matches = 0;

        //Extracting frames and loading it in current and previous images
        try{vid.frame_extraction(25);}
            catch(...){logger.log_warn("video ended ");return;}
            previous_frame = CaptureFrame(vid.retrieve_image(),"previous frame");
            try{vid.frame_extraction(25);}
            catch(...){logger.log_warn("video ended ");return;}
            current_frame = CaptureFrame(vid.retrieve_image(),"current frame");

        // stitching in loop and number of images are counted
        for(int image_count = 0; ; image_count++)
        {
            
            ORB_feature_points(current_frame,previous_frame);
            BF_matcher();
            find_homography();
            good_match_selection();
            matches = good_matches.size();

            //Stitch images only if more than 10 good matches are obtained. 4 is the theoratical minimum.
            if(matches > 10)
            {
                find_actual_homography();
                warp_image();
                image_blender();

                //Show live mosaic result 
                viewer.single_view_uninterrupted(mosaic_image,80);
                cv::waitKey(10);

                //start extracting frames for next iteration
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

        //Writing the mosaic image to disk
        cv::Mat output = mosaic_image.retrieve_image(); 
        cv::imwrite("Mosaic_image.jpg",output);
        logger.log_info("Image written to disk");

        return;
}
