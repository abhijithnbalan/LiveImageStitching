#include "capture_frame.h"
#include "view_frame.h"
int main(int argc, char** argv)
{
    ViewFrame viewer;
    CaptureFrame image1;
    image1.capture_image(argv[1],"First Image");
    viewer.single_view_interrupted(image1);
    return 0;
}