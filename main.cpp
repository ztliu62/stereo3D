#include <iostream>
#include <chrono>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

#include "StereoMatch.h"

using namespace std;

int main() {
    cv::Mat left = cv::imread("../images/PianoL/im0.png", 1);
    cv::Mat right = cv::imread("../images/PianoL/im1.png", 1);

    // OpenCV SGBM
    Mat disp;
    StereoOpenCV sgbm;
    cout << "Working on " << sgbm.getKindName() << endl;
    sgbm.setSADWindowSize(5);
    sgbm.setNumofDisparity(5*16);
    auto start = chrono::high_resolution_clock::now();

    sgbm.stereoMatch(left, right, disp);
    auto finish = chrono::high_resolution_clock::now();
    chrono::duration<double> elapsed = finish - start;
    cout << "Elapsed Time: " << elapsed.count() << endl;


    //self implemented SGBM
    Mat disp1;
    Stereo mySGBM;
    cout<< "Working on " << mySGBM.getKindName() << endl;
    mySGBM.setNumofDisparity(5*16);
    mySGBM.setCensusWindowSize(5);
    mySGBM.setP(8,56);
    mySGBM.setVisualize(true);
    start = chrono::high_resolution_clock::now();
    mySGBM.stereoMatch(right, left, disp1);
    finish = chrono::high_resolution_clock::now();
    elapsed = finish - start;
    cout << "Elapsed Time: " << elapsed.count() << endl;

    return 0;
}