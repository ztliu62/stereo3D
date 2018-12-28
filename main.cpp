#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "StereoMatch.h"

using namespace std;

int main() {
    cv::Mat left = cv::imread("../images/PianoL/im0.png", 1);
    cv::Mat right = cv::imread("../images/PianoL/im1.png", 1);

    Mat disp;
    StereoOpenCV sgbm;
    cout << sgbm.getKindName() << endl;
    sgbm.setSADWindowSize(5);
    sgbm.setNumofDisparity(5*16);
    sgbm.stereoMatch(left, right, disp);

    Mat disp1;
    Stereo mySGBM;
    cout<< mySGBM.getKindName() << endl;
    mySGBM.setNumofDisparity(5*16);
    mySGBM.setCensusWindowSize(5);
    mySGBM.setP(8,56);
    mySGBM.setVisualize(true);
    mySGBM.stereoMatch(right, left, disp1);

    return 0;
}