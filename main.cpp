#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/calib3d/calib3d.hpp>

using namespace std;

int main() {
    cout << "Hello, World!" << endl;
    cv::Mat left = cv::imread("../images/PianoL/im0.png", 1);
    cv::Mat right = cv::imread("../images/PianoL/im1.png", 1);
    cout << left.rows << " " << left.cols << endl;
    //cv::imshow("image", left);
    //cv::waitKey(0);

    cv::Mat leftgray, rightgray;
    leftgray.create(left.size(), CV_8UC1);
    rightgray.create(right.size(), CV_8UC1);

    cv::cvtColor(left, leftgray, CV_BGR2GRAY);
    cv::cvtColor(right, rightgray, CV_BGR2GRAY);

    int SADWinSize = 5;
    int numofDisparity = 5*16;
    int minDisparity = 0;
    int cn = 1;

    cv::Ptr<cv::StereoSGBM> sgbm = cv::StereoSGBM::create();
    sgbm->setPreFilterCap(63);
    sgbm->setBlockSize(SADWinSize);
    sgbm->setP1(8*cn*SADWinSize*SADWinSize);
    sgbm->setP2(32*cn*SADWinSize*SADWinSize);
    sgbm->setMinDisparity(minDisparity);
    sgbm->setNumDisparities(numofDisparity);
    sgbm->setUniquenessRatio(10);
    sgbm->setSpeckleWindowSize(100);
    sgbm->setSpeckleRange(32);
    sgbm->setDisp12MaxDiff(1);
    sgbm->setMode(cv::StereoSGBM::MODE_SGBM);

    cv::Mat disp, disp8;
    sgbm->compute(leftgray, rightgray, disp);
    disp.convertTo(disp8, CV_8U, 255/(numofDisparity*16.));

    cv::Mat disp_color;
    cv::applyColorMap(disp8, disp_color, cv::COLORMAP_JET);

    cv::imshow("Disparity", disp_color);
    cv::waitKey(0);

    return 0;
}