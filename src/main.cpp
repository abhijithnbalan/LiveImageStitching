#include "capture_frame.h"
#include "view_frame.h"
#include "image_mosaic.h"
#include <unistd.h>

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

    CaptureFrame ima1,ima2;
    ima1.capture_image(argv[1],"image1");
    ima2.capture_image(argv[2],"image2");
    logger.log_warn("Capturing images done");
    // mosaic.AKAZE_feature_match(ima1,ima2);

    mosaic.ORB_feature_match(ima1,ima2);
    logger.log_warn("feature detection done");
    mosaic.view_keypoints();
    logger.log_warn("Keypoint display done");
    mosaic.BF_matcher();
    logger.log_warn("feature matching done");
    mosaic.find_homography();
    logger.log_warn("homography done");
    mosaic.good_match_selection();
    logger.log_warn("goodmatch selection done");
    mosaic.warp_image();
    logger.log_warn("image warping done");
    mosaic.image_blender();
    logger.log_warn("blending done");




    // CaptureFrame image1,image2;
    // image1.capture_image(argv[1],"First Image");
    // image2.capture_image(argv[2],"Second Image");
    // // viewer.single_view_interrupted(image1);
    // // mosaic.AKAZE_feature_match(image1,image2);
    // mosaic.Opencv_Stitcher(image1,image2);
    // viewer.single_view_interrupted(mosaic.mosaic_image);

    // CaptureFrame video;
    // video.capture_video(1,"input video");
    // mosaic.video_mosaic(video);

    // CaptureFrame video;
    // video.capture_video(argv[1],"input video");
    // mosaic.image_stream_recorder(video,11);
    // std::cout<<"image recording done\n";
    // mosaic.native_stitcher();



    return 0;
}