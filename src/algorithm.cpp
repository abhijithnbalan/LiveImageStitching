

#include "algorithm.h"
#include <opencv2/opencv.hpp>

CaptureFrame Algorithm::CLAHE_dehaze(CaptureFrame object) //CLAHE based basic dehazing algorithm
{
    cv::Mat segmented;
    segmented = CLAHE_dehaze(object.retrieve_image());
    CaptureFrame output(segmented, "Dehazed image");
    return output;
}
cv::Mat Algorithm::CLAHE_dehaze(cv::Mat object) //CLAHE based basic dehazing algorithm
{
    cv::Mat image_hsv;
    cvtColor(object, image_hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    split(image_hsv, channels);
    cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE();
    clahe->setClipLimit(CLAHE_clip_limit);
    clahe->apply(channels[2], channels[2]);
    clahe->apply(channels[1], channels[1]);
    channels[2] = channels[2] * 0.85;
    merge(channels, image_hsv);
    cv::Mat dehazed;
    cvtColor(image_hsv, dehazed, cv::COLOR_HSV2BGR);
    // GaussianBlur(dehazed, dehazed, cv::Size(3, 3), 2, 2);
    return dehazed;
}
CaptureFrame Algorithm::hist_equalize(CaptureFrame object) //CLAHE based basic dehazing algorithm
{
    cv::Mat image_hsv;
    cvtColor(object.retrieve_image(), image_hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> channels;
    split(image_hsv, channels);
    cv::equalizeHist(channels[1], channels[1]);
    channels[2] = channels[2] * 0.85;
    merge(channels, image_hsv);
    cv::Mat dehazed;
    cvtColor(image_hsv, dehazed, cv::COLOR_HSV2BGR);
    // GaussianBlur(dehazed, dehazed, cv::Size(3, 3), 2, 2);
    CaptureFrame output(dehazed, "Dehazed image");
    return output;
}

void Algorithm::set_CLAHE_clip_limit(int clip_limit)//function which set the clip limit in CLAHE dehazing
{
    CLAHE_clip_limit = clip_limit;
}
Algorithm::Algorithm()
{
    CLAHE_clip_limit = 2;
}