#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "StereoMatch.h"

using namespace std;

int main() {
    cout << "Hello, World!" << endl;
    cv::Mat left = cv::imread("../images/PianoL/im0.png", 1);
    cv::Mat right = cv::imread("../images/PianoL/im1.png", 1);
    cout << left.rows << " " << left.cols << endl;

    Mat disp;
    //cv::imshow("image", left);
    //cv::waitKey(0);
    StereoOpenCV sgbm;
    cout << sgbm.getKindName() << endl;
    cout << sgbm.getParamCount() << endl;
    sgbm.stereomatch(left, right, disp);

    return 0;
}