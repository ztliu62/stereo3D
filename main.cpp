#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "StereoMatch.h"

using namespace std;

int main() {
    cout << "Hello, World!" << endl;
    cv::Mat left = cv::imread("../images/Motorcycle/im0.png", 1);
    cv::Mat right = cv::imread("../images/Motorcycle/im1.png", 1);
    cout << left.rows << " " << left.cols << endl;

    //cv::imshow("image", left);
    //cv::waitKey(0);

    Mat disp;
    /*
    StereoOpenCV sgbm;
    cout << sgbm.getKindName() << endl;
    cout << sgbm.getParamCount() << endl;
    sgbm.stereomatch(left, right, disp);
    */
    Stereo mySGBM;
    cout<< mySGBM.getKindName() << endl;
    cout<< mySGBM.getParamCount() << endl;
    mySGBM.stereomatch(right, left, disp);

    return 0;
}