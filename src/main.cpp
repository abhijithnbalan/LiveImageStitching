#include "capture_frame.h"
#include "view_frame.h"
#include "image_mosaic.h"
#include <unistd.h>
#include <string.h>
#include <sstream>

int main(int argc, char **argv) //The main Function
{
    int success = chdir("..");
    Logger logger;
    logger.log_warn("Image Mosaicing starts");
    logger.log_debug("Working directory switched to one directory back");
   
    //Changing directory for accessing files. another workaround is giving full path for each files.
    
    if(success != 0)
    {
        logger.log_error("Unable to change working directory");
        std::cout<<"couldn't change the directory/\n";
        return -1;    
    } 
    ImageMosaic mosaic;
    mosaic.use_dehaze = true;
    ViewFrame viewer;
    char* mode = argv[1];
    
    

    if(std::string(argv[1]) == "pair")
    {
        argv = argv + 1;
        logger.log_warn("image pair mode");
        CaptureFrame ima1,ima2;

        ima1.capture_image(argv[1],"image1");
        ima2.capture_image(argv[2],"image2");
        logger.log_info("Capturing images successful");
    
        mosaic.algo.AKAZE_feature_points(ima1,ima2);
        logger.log_info("feature detection successful");
        mosaic.view_keypoints();
        logger.log_info("Keypoint display successful");
        mosaic.algo.BF_matcher();
        logger.log_info("feature matching successful");
        mosaic.find_homography();
        logger.log_info("homography successful");
        mosaic.good_match_selection();
        logger.log_info("goodmatch selection successful");
        mosaic.find_actual_homography();
        logger.log_info("homography according to good matches");
        mosaic.warp_image();
        logger.log_info("image warping successful");
        mosaic.image_blender();
        logger.log_info("blending successful");
        viewer.single_view_interrupted(mosaic.mosaic_image);
        cv::waitKey(15);
    }




    // CaptureFrame image1,image2;
    // image1.capture_image(argv[1],"First Image");
    // image2.capture_image(argv[2],"Second Image");
    else if(std::string(argv[1]) == "img_vec_img")
    {
        argv = argv + 1;
        logger.log_warn("image vector recording with arguments");
        argv[1] == "";
        mosaic.image_vector_maker(argc, argv);
        mosaic.Opencv_Stitcher();
        viewer.single_view_interrupted(mosaic.mosaic_image);
    }

    else if(std::string(argv[1]) == "img_vec_vid")
    {
        argv = argv + 1;
        logger.log_warn("image vector recording with video");
        CaptureFrame video;
        video.capture_video(argv[2],"input video");
        mosaic.image_stream_recorder(video,11);
        logger.log_info("Image stream recording completed");
        mosaic.Opencv_Stitcher();
    }
    
    
    if(std::string(argv[1]) == "live")
    {
        argv = argv + 1;
        logger.log_warn("live mosaic mode");
        CaptureFrame vid;
        std::istringstream ss(argv[1]);
        int camera_port;
        if (!(ss >> camera_port))
        {
            vid.capture_video(argv[1],"video input");
            mosaic.roi_x = 50;
            mosaic.roi_y = 60;
            mosaic.roi_width = 80;
            mosaic.roi_height = 70;
            mosaic.live_mosaicing_video(vid);
        }
        else
        {
            // mosaic.roi_x = 50;
            // mosaic.roi_y = 50;
            // mosaic.roi_width = 70;
            // mosaic.roi_height = 70;
            vid.capture_video(camera_port,"camera input");
            mosaic.live_mosaicing_camera(vid);
        }
        // mosaic.image_vector_maker(vid);
        // mosaic.display_image_vector();
        // mosaic.Opencv_Stitcher();
        
        logger.log_info("Mosaicing Completed ");
        viewer.single_view_interrupted(mosaic.mosaic_image);
        //Writing the mosaic image to disk
        if(mosaic.mosaic_image.retrieve_image().data)
        {
            CaptureFrame output = mosaic.crop_live(); 
            cv::imwrite("Mosaic_image.jpg",output.retrieve_image());
            cv::imshow("output is the is of the",output.retrieve_image());
            logger.log_info("Image written to disk");
        }
        cv::waitKey(15);
    }
    return 0;
}


