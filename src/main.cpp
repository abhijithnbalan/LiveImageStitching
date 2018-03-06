#include "capture_frame.h"
#include "view_frame.h"
#include "image_mosaic.h"
#include <unistd.h>
#include <string.h>

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
    ViewFrame viewer;
    char* mode = argv[1];
    
    

    if(std::string(argv[1]) == "pair")
    {
        argv = argv + 1;
        logger.log_warn("image pair mode");
        CaptureFrame ima1,ima2;
        ima1.capture_image(argv[1],"image1");
        ima2.capture_image(argv[2],"image2");
        logger.log_warn("Capturing images done");
    
        mosaic.ORB_feature_points(ima1,ima2);
        logger.log_warn("feature detection done");
        mosaic.view_keypoints();
        logger.log_warn("Keypoint display done");
        mosaic.BF_matcher();
        logger.log_warn("feature matching done");
        mosaic.find_homography();
        logger.log_warn("homography done");
        mosaic.good_match_selection();
        logger.log_warn("goodmatch selection done");
        mosaic.find_actual_homography();
        logger.log_warn("homography according to good matches");
        mosaic.warp_image();
        logger.log_warn("image warping done");
        mosaic.image_blender();
        logger.log_warn("blending done");
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
    
    std::cout<<argv[1]<<"\n";
    if(std::string(argv[1]) == "live")
    {
        argv = argv + 1;
        logger.log_warn("live mosaic mode");
        CaptureFrame vid;
        std::cout<<argv[1]<<"\n";
        vid.capture_video(argv[1],"video input");
        // mosaic.image_vector_maker(vid);
        // mosaic.display_image_vector();
        // mosaic.Opencv_Stitcher();
        mosaic.live_mosaicing(vid);
        logger.log_warn("Mosaicing Completed ");
        viewer.single_view_interrupted(mosaic.mosaic_image);
        cv::waitKey(15);
    }
    return 0;
}