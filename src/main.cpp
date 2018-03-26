#include "capture_frame.h"
#include "view_frame.h"
#include "image_mosaic.h"
#include <unistd.h>
#include <string.h>
#include <sstream>
#include <timer.h>

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
    
    
    Timer timer1,timer2,timer3,timer4,timer5,timer6;
    if(std::string(argv[1]) == "pair")
    {
        argv = argv + 1;
        logger.log_warn("image pair mode");
        CaptureFrame ima1,ima2;
        ima1.capture_image(argv[1],"image1");
        ima2.capture_image(argv[2],"image2");
        logger.log_warn("Capturing images done");

        timer1.timer_init();
        mosaic.algo.ORB_feature_points(ima1,ima2);
        timer1.timer_end();
        logger.log_warn("feature detection done");
        // mosaic.view_keypoints();
        // logger.log_warn("Keypoint display done");

        printf("number of keypoints img1 : %d  img2 : %d  \n",static_cast<int>(mosaic.algo.keypoints_current_image.size()),static_cast<int>(mosaic.algo.keypoints_previous_image.size()));

        timer2.timer_init();
        mosaic.algo.BF_matcher();
        timer2.timer_end();
        logger.log_warn("feature matching done");

        timer3.timer_init();
        mosaic.find_homography();
        timer3.timer_end();
        logger.log_warn("homography done");

        timer4.timer_init();
        mosaic.good_match_selection();
        timer4.timer_end();
        logger.log_warn("goodmatch selection done");
        mosaic.find_actual_homography();
        logger.log_warn("homography according to good matches");

        timer5.timer_init();
        mosaic.warp_image();
        timer5.timer_end();
        logger.log_warn("image warping done");

        timer6.timer_init();
        mosaic.image_blender();
        timer6.timer_end();
        logger.log_warn("blending done");

        printf("keypoint %f matcher %f homogra %f goodones %f warp %f blender %f\n",timer1.execution_time*1000,timer2.execution_time*1000,timer3.execution_time*1000,timer4.execution_time*1000,timer5.execution_time*1000,timer6.execution_time*1000);
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
        std::istringstream ss(argv[1]);
        int camera_port;
        if (!(ss >> camera_port))
        {
            vid.capture_video(argv[1],"video input");
            mosaic.live_mosaicing_video(vid);
        }
        else
        {
            vid.capture_video(camera_port,"camera input");
            mosaic.live_mosaicing_camera(vid);
        }
        // mosaic.image_vector_maker(vid);
        // mosaic.display_image_vector();
        // mosaic.Opencv_Stitcher();
        
        logger.log_warn("Mosaicing Completed ");
        viewer.single_view_interrupted(mosaic.mosaic_image);
        //Writing the mosaic image to disk
        cv::Mat output = mosaic.mosaic_image.retrieve_image(); 
        cv::imwrite("Mosaic_image.jpg",output);
        logger.log_info("Image written to disk");
        cv::waitKey(15);
    }
    return 0;
}


