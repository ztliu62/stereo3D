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
    this->params[0] = 9; // SAD Windows Size
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
    this->paramCount = 5;
    this->params[0] = 5; //Census Window Size
    this->params[1] = 5*16; // Number of Disparity
    this->params[2] = 0; // min disparity
    this->params[3] = 8; //P1
    this->params[4] = 32; //P2
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
    //Stereo::stereoCensus(leftgray, rightgray, disp);
    Stereo::stereoSGBM(leftgray, rightgray, disp);


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

    cv::medianBlur(disp, disp, 3);
    imwrite("Census.jpg", disp);
    //display code
    cv::Mat disp_color;
    cv::applyColorMap(disp, disp_color, cv::COLORMAP_JET);

    cv::imshow("Disparity", disp_color);
    cv::waitKey(0);
    //cout<< disp << endl;

}

void Stereo::stereoSGBM(Mat &left, Mat &right, Mat &disp) {
    int w = this->params[0];
    int MaxDisp = this->params[1];
    const int width = left.cols, height = left.rows;
    int P1 = this->params[3];
    int P2 = this->params[4];

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

    cv::medianBlur(disp, disp, 3);
    imwrite("Census.jpg", disp);
    /*
    //display code
    cv::Mat disp_color;
    cv::applyColorMap(disp, disp_color, cv::COLORMAP_JET);

    cv::imshow("Disparity", disp_color);
    cv::waitKey(0);
    //cout<< disp << endl;
    */

    //
    // Starting SGBM
    //
    cout << "Starting SGBM ..." << endl;
    cv::Mat disp1;
    disp1.create(height, width, CV_8U);

    int LrWidth = width + 2;
    int LrMax = MaxDisp + 2;


    int ****Lr;
    Lr = new int***[2];
    for (int row = 0; row < 2; ++row) {
        Lr[row] = new int**[LrWidth];
        for (int col = 0; col < LrWidth; ++col) {
            Lr[row][col] = new int*[LrMax];
            for (int d = 0; d < LrMax; d++){
                Lr[row][col][d] = new int[4](); //four directions: (-1, 0), (-1,-1),(0,-1),(1,-1)
            }
        }
    }

    int ***minLr;
    minLr = new int **[2];
    for (int row = 0; row < 2; row++){
        minLr[row] = new int*[LrWidth];
        for (int col = 0; col < LrWidth; col++){
            minLr[row][col] = new int[4]();
        }
    }

    long ***S;
    S = new long**[height];
    for (int row = 0; row < height; row++){
        S[row] = new long*[width];
        for (int col = 0; col < width; col++){
            S[row][col] = new long[MaxDisp]();
        }
    }

    // Initialize Lr
    for (int i = 0; i < 2; i++){
        for (int j = 0; j < LrWidth; j++){
            for(int d = 0; d < LrMax; d++){
                for (int dir = 0; dir < 4; dir++){
                    if (d == 0 || d == LrMax-1){
                        Lr[i][j][d][dir] = 10000;
                    } else {
                        Lr[i][j][d][dir] = 0;
                    }
                }
            }
        }
    }
    // Initialize minLr
    for (int i = 0; i < 2; i++){
        for(int j = 0; j < LrWidth; j++){
            for (int dir = 0; dir < 4; dir++){
                minLr[i][j][dir] = 0;
            }
        }
    }
    // Initialize S
    for (int i = 0; i < height; i++){
        for (int j = 0; j < width; j++){
            for (int d = 0; d < MaxDisp; d++){
                S[i][j][d] = 0;
            }
        }
    }

    // potential passes loop:  for (int p = 0; p < passes; p++)
    for (int i = 0; i < height; i++){
        for (int j = 1; j < LrWidth -1; j++){
            int minLr0 = INT_MAX, minLr1 = INT_MAX, minLr2 = INT_MAX, minLr3 = INT_MAX;
            for (int d = 1; d < LrMax-1; d++){
                int Lr0 = (int)Cost[i][j-1][d-1] + min(min(Lr[0][j-1][d][0], Lr[0][j-1][d-1][0]+P1),
                                                        min(Lr[0][j-1][d+1][0] + P1, minLr[0][j-1][0] + P2)) - minLr[0][j-1][0];
                int Lr1 = (int)Cost[i][j-1][d-1] + min(min(Lr[1][j-1][d][1], Lr[1][j-1][d-1][1]+P1),
                                                       min(Lr[1][j-1][d+1][1] + P1, minLr[1][j-1][1] + P2)) - minLr[1][j-1][1];
                int Lr2= (int)Cost[i][j-1][d-1] + min(min(Lr[1][j][d][2], Lr[1][j][d-1][2]+P1),
                                                       min(Lr[1][j][d+1][2] + P1, minLr[1][j][2] + P2)) - minLr[1][j][2];
                int Lr3 = (int)Cost[i][j-1][d-1] + min(min(Lr[1][j+1][d][3], Lr[1][j+1][d-1][3]+P1),
                                                       min(Lr[1][j+1][d+1][3] + P1, minLr[1][j+1][3] + P2)) - minLr[1][j+1][3];
                Lr[0][j][d][0] = Lr0;
                minLr0 = min(minLr0, Lr0);

                Lr[0][j][d][1] = Lr1;
                minLr1 = min(minLr1, Lr1);

                Lr[0][j][d][2] = Lr2;
                minLr2 = min(minLr2, Lr2);

                Lr[0][j][d][3] = Lr3;
                minLr3 = min(minLr3, Lr3);

                S[i][j-1][d-1] = (Lr0 + Lr1 + Lr2 + Lr3)/4;
                //cout << S[i][j-1][d-1] << endl;
            }
            minLr[0][j][0] = minLr0;
            minLr[0][j][1] = minLr1;
            minLr[0][j][2] = minLr2;
            minLr[0][j][3] = minLr3;

        }
        swap(Lr[0], Lr[1]);
        swap(minLr[0], minLr[1]);
    }



    for(int i = height - 1; i >= 0; i--){
        for(int j = width; j > 0; j--){
            int minLr01 = INT_MAX, minLr11 = INT_MAX, minLr21 = INT_MAX, minLr31 = INT_MAX;
            for (int d = 1; d < LrMax-1 && d <= j; d++){
                int Lr01 = (int)Cost[i][j-1][d-1] + min(min(Lr[1][j+1][d][0], Lr[1][j+1][d-1][0]+P1),
                                                       min(Lr[1][j+1][d+1][0] + P1, minLr[1][j+1][0] + P2)) - minLr[1][j+1][0];
                int Lr11 = (int)Cost[i][j-1][d-1] + min(min(Lr[0][j-1][d][1], Lr[0][j-1][d-1][1]+P1),
                                                       min(Lr[0][j-1][d+1][1] + P1, minLr[0][j-1][1] + P2)) - minLr[0][j-1][1];
                int Lr21 = (int)Cost[i][j-1][d-1] + min(min(Lr[0][j][d][2], Lr[0][j][d-1][2]+P1),
                                                       min(Lr[0][j][d+1][2] + P1, minLr[0][j][2] + P2)) - minLr[0][j][2];
                int Lr31 = (int)Cost[i][j-1][d-1] + min(min(Lr[0][j+1][d][3], Lr[0][j+1][d-1][3]+P1),
                                                       min(Lr[0][j+1][d+1][3] + P1, minLr[0][j+1][3] + P2)) - minLr[0][j+1][3];

                Lr[1][j][d][0] = Lr01;
                minLr01 = min(minLr01, Lr01);

                Lr[1][j][d][1] = Lr11;
                minLr11 = min(minLr11, Lr11);

                Lr[1][j][d][2] = Lr21;
                minLr21 = min(minLr21, Lr21);

                Lr[1][j][d][3] = Lr31;
                minLr31 = min(minLr31, Lr31);

                S[i][j-1][d-1] += (Lr01 + Lr11 + Lr21 + Lr31)/4;
            }
            minLr[1][j][0] = minLr01;
            minLr[1][j][1] = minLr11;
            minLr[1][j][2] = minLr21;
            minLr[1][j][3] = minLr31;
        }
        swap(Lr[0], Lr[1]);
        swap(minLr[0], minLr[1]);
    }


    for (int i = 0; i < height; i++){
        for (int j = 0; j < width; j++){
            long minSum = S[i][j][0], minDis = 0;
            //cout << minSum << endl;
            for (int d = 0; d < MaxDisp; d++){
                if (S[i][j][d] < minSum){
                    //cout << S[i][j][d] << endl;
                    minSum = S[i][j][d];
                    minDis = d;
                }
            }

            disp1.at<uchar>(i,j)  =  minDis*255.0/MaxDisp;
        }
    }
    //cout << disp1 << endl;
    cv::medianBlur(disp1, disp1, 3);
    imwrite("SGBM.jpg", disp1);

    cv::Mat disp_color;
    cv::applyColorMap(disp1, disp_color, cv::COLORMAP_JET);

    cv::imshow("Disparity", disp_color);
    cv::waitKey(0);

    delete []Lr;
    delete []minLr;
    delete []S;
}