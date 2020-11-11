#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <iostream>

using namespace std;
using namespace cv;
Mat src_gray;
int main()
{
    VideoCapture cap(0);

    if (!cap.open(0)) return 0;

    while (true)
    {
        Mat frame;
        cap >> frame;

        if (waitKey(30) == 27 || frame.empty()) break;

        imshow("Hough Circle Transform Demo", frame);
    }

    return 0;
}