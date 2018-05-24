#include "capture_frame.h"
#include "view_frame.h"
#include "image_mosaic.h"
#include <unistd.h>
#include <string.h>
#include <sstream>

//for json support -- rapidjason is used
#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/ostreamwrapper.h"
#include "rapidjson/writer.h"
#include <fstream>

int main(int argc, char **argv) //The main Function
{

    bool debug_mode, use_dehaze, use_same_location, dev_mode;
    std::string running_mode, execution_mode;
    int roi_x, roi_y, roi_width, roi_height;
    std::string default_intermediate_output_location, default_output_location;

    int video_frame_skip, camera_delay_between_frames;

    int success = chdir("..");
    Logger logger;
    logger.log_warn("Image Mosaicing starts");
    logger.log_debug("Working directory switched to one directory back");

    //Changing directory for accessing files. another workaround is giving full path for each files.

    if (success != 0)
    {
        logger.log_error("Unable to change working directory");
        std::cout << "couldn't change the directory/\n";
        return -1;
    }

    try
    {
        std::ifstream ifs("configure.json");

        if (ifs.fail())
            throw(20);
        rapidjson::IStreamWrapper isw(ifs);
        rapidjson::Document configuration_file;
        configuration_file.ParseStream(isw);

        debug_mode = configuration_file["Program Execution"]["DebugMode"].GetBool();
        running_mode = configuration_file["Program Execution"]["RunningMode"].GetString(); //Reading data from json
        execution_mode = configuration_file["Program Execution"]["ExeMode"].GetString();

        roi_x = configuration_file["General Settings"]["RegionOfInterest_x"].GetInt();
        roi_y = configuration_file["General Settings"]["RegionOfInterest_y"].GetInt();
        roi_width = configuration_file["General Settings"]["RegionOfInterest_width"].GetInt();
        roi_height = configuration_file["General Settings"]["RegionOfInterest_height"].GetInt();
        use_dehaze = configuration_file["General Settings"]["UseDehaze(CLAHE)"].GetBool();
        use_same_location = configuration_file["General Settings"]["UseSameLocation"].GetBool();

        camera_delay_between_frames = configuration_file["Live Mosaic"]["CameraDelay"].GetInt();
        video_frame_skip = configuration_file["Live Mosaic"]["VideoFrameSkip"].GetFloat();
        default_intermediate_output_location = configuration_file["Live Mosaic"]["DefaultIntermediateOutput"].GetString();
        default_output_location = configuration_file["Live Mosaic"]["DefaultOutput"].GetString();
    }
    catch (...)
    {
        logger.log_error("Unable to open or read json file");
        std::cout << "Error in file opening or reading \"laser_calibration_values.json\" file\n";
        exit(0);
    }

    printf("\n\n\t\t\tConfiguration File\n\n"
           "Execution Settings\n"
           "Running the Program in %s in %s with Debug status : %d \n\n"
           "General Settings\n"
           "The Region of Interset is selected as (%d,%d,%d,%d)\n"
           "Same Location Status : %d\n"
           "Dehaze Status %d\n\n"
           "Live Mosaic Settings\n"
           "Camera's delay between frames is taken as %d\n"
           "%d Frames will be skipped before taking the next frame\n"
           "The intermediate outputs will be written at : %s\n "
           "The output will be written at : %s\n\n",
           running_mode.c_str(), execution_mode.c_str(), debug_mode, roi_x, roi_y, roi_width, roi_height, use_same_location, use_dehaze, camera_delay_between_frames, video_frame_skip, default_intermediate_output_location.c_str(), default_output_location.c_str());

    logger.debug_mode = debug_mode;
    ImageMosaic mosaic;
    mosaic.video_frame_skip = video_frame_skip;
    mosaic.camera_frame_delay = camera_delay_between_frames;

    mosaic.roi_x = roi_x;
    mosaic.roi_y = roi_y;
    mosaic.roi_width = roi_width;
    mosaic.roi_height = roi_height;
    std::string file_loc(argv[2]);
    if (use_same_location)
    {
        mosaic.output_filename = file_loc;
        mosaic.intermediate_filename = file_loc;
        
    }
    else
    {
        mosaic.output_filename = default_output_location;
        mosaic.intermediate_filename = default_intermediate_output_location;
    }
    mosaic.use_dehaze = use_dehaze;
    ViewFrame viewer;

    if (execution_mode == "DEV")
        dev_mode = true;
    else
        dev_mode = false;
    mosaic.dev_mode = dev_mode;

    if (running_mode == "commandline")
    {
        char *mode = argv[1];

        if (std::string(argv[1]) == "pair")
        {
            argv = argv + 1;
            logger.log_warn("image pair mode");
            CaptureFrame ima1, ima2;

            ima1.capture_image(argv[1], "image1");
            ima2.capture_image(argv[2], "image2");
            logger.log_info("Capturing images successful");

            mosaic.algo.AKAZE_feature_points(ima1, ima2);
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
            if (dev_mode)
            {
                viewer.single_view_interrupted(mosaic.mosaic_image);
            }
            cv::waitKey(15);
        }

        else if (std::string(argv[1]) == "img_vec_img")
        {
            argv = argv + 1;
            logger.log_info("image vector recording with arguments");
            argv[1] == "";
            mosaic.image_vector_maker(argc, argv);
            mosaic.Opencv_Stitcher();
            if (dev_mode)
            {
                viewer.single_view_interrupted(mosaic.mosaic_image);
            }
        }

        else if (std::string(argv[1]) == "img_vec_vid")
        {
            argv = argv + 1;
            logger.log_info("image vector recording with video");
            CaptureFrame video;
            video.capture_video(argv[2], "input video");
            mosaic.image_stream_recorder(video, 11);
            logger.log_info("Image stream recording completed");
            mosaic.Opencv_Stitcher();
            if (dev_mode)
            {
                viewer.single_view_interrupted(mosaic.mosaic_image);
            }
        }

        else if (std::string(argv[1]) == "live")
        {
            argv = argv + 1;
            logger.log_warn("live mosaic mode");
            CaptureFrame vid;
            std::istringstream ss(argv[1]);
            int camera_port;

            if (!(ss >> camera_port))
            {
                vid.capture_video(argv[1], "video input");

                mosaic.live_mosaicing_video(vid);
            }
            else
            {

                vid.capture_video(camera_port, "camera input");
                mosaic.live_mosaicing_camera(vid);
            }

            logger.log_info("Mosaicing Completed ");
            if (dev_mode)
            {
                viewer.single_view_interrupted(mosaic.mosaic_image);
            } //Writing the mosaic image to disk
            if (mosaic.mosaic_image.retrieve_image().data)
            {
                CaptureFrame output = mosaic.crop_live();
                std::string file_prename = mosaic.output_filename.substr(0, mosaic.output_filename.size() - 4);

                cv::imwrite(file_prename + "_mosaic.jpg", output.retrieve_image());
                if (dev_mode)
                {
                    cv::imshow("output is the is of the", output.retrieve_image());
                }
                logger.log_info("Image written to disk");
            }
            cv::waitKey(15);
        }
        else
        {
            logger.log_error("Invalid program mode.\n Please provide one in ( pair , live , img_vec_img , img_vec_vid )\n");
            return 0;
        }
    }
    else if (running_mode == "GUI")
    {
        //GUI code goes here
    }

    else if (running_mode == "automated")
    {
        //Automated code goes here
    }
    else
    {
        logger.log_error("Invalide Running mode.\n Please provide one in ( commandline , GUI , automated )\n");
        return 0;
    }
    return 1;
}
