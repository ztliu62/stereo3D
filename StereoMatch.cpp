//
// Created by jerry on 12/24/18.
//
#include <iostream>

#include "StereoMatch.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

StereoOpenCV::StereoOpenCV() {
    this->paramCount = 3;
    this->params[0] = 11; // SAD Windows Size
    this->params[1] = 5*16; // Number of Disparity
    this->params[2] = 0; // min Disparity
}

string StereoOpenCV::getKindName() {
    return "SGBM_OpenCV";
}

int StereoOpenCV::getParamCount() {
    return this->paramCount;
}

void StereoOpenCV::setParamValue(int index, int value) {

    if (index == 0){
        if (value%2 == 0){
            value += 1;
        }
    } else if (index == 1){
        if (value%16 != 0){
            value = (value/16)*16;
        }
    } else if (index == 2){

    } else {
        return;
    }
    this->params[index] = value;
}

void StereoOpenCV::stereomatch(Mat &left, Mat &right, Mat &disp){
    Mat leftgray, rightgray;
    leftgray.create(left.size(), CV_8UC1);
    rightgray.create(right.size(), CV_8UC1);
    cvtColor(left, leftgray, CV_BGR2GRAY);
    cvtColor(right, rightgray, CV_BGR2GRAY);

    int SADWinSize = this->params[0];
    int numofDisparity = this->params[1];
    int minDisparity = this->params[2];
    int cn = 1;

    Ptr<StereoSGBM> sgbm = StereoSGBM::create();
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
    sgbm->setMode(StereoSGBM::MODE_SGBM);

    Mat disp1;
    sgbm->compute(leftgray, rightgray, disp1);
    disp1.convertTo(disp, CV_8U, 255/(numofDisparity*16.));

    cv::Mat disp_color;
    cv::applyColorMap(disp, disp_color, cv::COLORMAP_JET);

    cv::imshow("Disparity", disp_color);
    cv::waitKey(0);
}

Stereo::Stereo(){
    this->paramCount = 3;
    this->params[0] = 9; //Census Window Size
    this->params[1] = 5*16; // Number of Disparity
    this->params[2] = 0; // min disparity
}

string Stereo::getKindName() {
    return "mySGBM";
}

int Stereo::getParamCount() {
    return this->paramCount;
}

void Stereo::setParamValue(int index, int value) {

    if (index == 0){
        if (value%2 == 0){
            value += 1;
        }
    } else if (index == 1){
        if (value%16 != 0){
            value = (value/16)*16;
        }
    } else if (index == 2){

    } else {
        return;
    }
    this->params[index] = value;
}

void Stereo::stereomatch(Mat &left, Mat &right, Mat &disp) {
    Mat leftgray, rightgray;
    leftgray.create(left.size(), CV_8UC1);
    rightgray.create(right.size(), CV_8UC1);
    cvtColor(left, leftgray, CV_BGR2GRAY);
    cvtColor(right, rightgray, CV_BGR2GRAY);

    int CensusWindowSize = this->params[0];
    int numofDisparity = this->params[1];
    int minDisparity = this->params[2];


    Mat disp1;
    Stereo::stereoCensus(leftgray, rightgray, disp);
    //disp1.convertTo(disp, CV_8U, 255/(numofDisparity*16.));

}

void Stereo::stereoCensus(Mat &left, Mat &right, Mat &disp) {
    int w = this->params[0];
    int MaxDisp = this->params[1];
    const int width = left.cols, height = left.rows;

    disp = Mat::zeros(height, width, CV_8U);

    long CensusLeft[height][width];
    long CensusRight[height][width];

    int bit = 0;
    cout << "Applying Census Transformation ... " << endl;
    for (int i = w/2; i < height-w/2; i++){
        for (int j = w/2; j < width-w/2; j++){
            long bitLeft = 0, bitRight = 0;
            for (int row = i-w/2; row <= i+w/2; row++){
                for(int col = j-w/2; col <= j+w/2; col++){
                    if (row == i && col == j){
                        continue;
                    }
                    bitLeft <<=1;
                    if (left.at<uchar>(row, col) < left.at<uchar>(i,j)){
                        bit = 1;
                    } else {
                        bit = 0;
                    }
                    bitLeft |= bit;

                    int rcenter = (int)right.at<uchar>(i, j);
                    bitRight <<= 1;
                    if (right.at<uchar>(row, col) < right.at<uchar>(i, j)){
                        bit = 1;
                    } else {
                        bit = 0;
                    }
                    bitRight |= bit;

                }
            }

            CensusLeft[i][j] = bitLeft;
            CensusRight[i][j] = bitRight;
        }
    }
    //imwrite("CensusLeft.jpg", CensusLeft);
    //imwrite("CensusRight.jpg", CensusRight);

    //Mat Cost = Mat::zeros(height, width*MaxDisp, CV_32S);
    unsigned long ***Cost;
    Cost = new unsigned long**[left.rows];
    for (int row = 0; row < left.rows; ++row) {
        Cost[row] = new unsigned long*[left.cols];
        for (int col = 0; col < left.cols; ++col) {
            Cost[row][col] = new unsigned long[MaxDisp]();
        }
    }


    cout << "Finding Hamming Distance ... " << endl;
    for(int i = w/2; i < height-w/2; i++){
        for (int j = w/2; j < width - w/2; j++){
            long LeftVal = 0, RightVal = 0;
            for (int  d = 0 ; d< MaxDisp; d++){
                LeftVal = CensusLeft[i][j];
                if (j+d < width-w/2){
                    RightVal = CensusRight[i][j+d];
                } else {
                    RightVal = CensusRight[i][j+d-MaxDisp];
                }
                //cout << LeftVal << " " << RightVal << endl;

                long ans = LeftVal^RightVal;
                int dist = 0;
                while (ans){
                    dist++;
                    ans &= ans-1;
                }
                //Cost.at<int>(i, j*MaxDisp+d)= dist;
                Cost[i][j][d] = dist;
            }

        }
    }
    //cout << Cost << endl;

    for (int row = 0; row < height; row++){
        for (int col = 0; col < width; col++){
            long minCost = Cost[row][col][0];
            int minDisp = 0;
            for(int d = MaxDisp - 1; d >= 0; d--){
                if (Cost[row][col][d] < minCost){
                    //cout << Cost.at<int>(row, col*MaxDisp+d) << endl;
                    minCost = Cost[row][col][d];
                    minDisp = d;
                }
            }
            disp.at<uchar>(row, col) = minDisp*255.0/MaxDisp;
        }
    }
    imwrite("Census.jpg", disp);
    cout<< disp << endl;

}