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
    this->SADWindowSize = 5;
    this->NumofDisparity = 5*16;
}

string StereoOpenCV::getKindName() {
    return "SGBM_OpenCV";
}

void StereoOpenCV::setNumofDisparity(int value) {
    if (value%16 != 0){
        this->NumofDisparity = (value/16)*16;
    }
}

void StereoOpenCV::setSADWindowSize(int value) {
    this->SADWindowSize = value|1;
}

void StereoOpenCV::stereoMatch(Mat &left, Mat &right, Mat &disp){
    Mat leftgray, rightgray;
    leftgray.create(left.size(), CV_8UC1);
    rightgray.create(right.size(), CV_8UC1);
    cvtColor(left, leftgray, CV_BGR2GRAY);
    cvtColor(right, rightgray, CV_BGR2GRAY);

    int SADWinSize = this->SADWindowSize;
    int numofDisparity = this->NumofDisparity;
    int minDisparity = 0;
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
    this->NumofDisparity = 5*16;
    this->CensusWindowSize = 7;
    this->P1 = 8;
    this->P2 = 32;
    this->Visualize = true;
}

string Stereo::getKindName() {
    return "mySGBM";
}

void Stereo::setCensusWindowSize(int value) {
    this->CensusWindowSize = value|1;
}

void Stereo::setNumofDisparity(int value) {
    if (value%16 != 0){
        this->NumofDisparity = (value/16)*16;
    }
}

void Stereo::setP(int Param1, int Param2) {
    this->P1 = Param1;
    this->P2 = Param2;
}

void Stereo::setVisualize(bool value){
    this->Visualize = value;
}


void Stereo::stereoMatch(Mat &left, Mat &right, Mat &disp) {
    Mat leftgray, rightgray;
    leftgray.create(left.size(), CV_8UC1);
    rightgray.create(right.size(), CV_8UC1);

    cvtColor(left, leftgray, CV_BGR2GRAY);
    cvtColor(right, rightgray, CV_BGR2GRAY);

    width = leftgray.cols, height = leftgray.rows;


    unsigned long ***Cost;
    Cost = new unsigned long**[height];
    for (int row = 0; row < height; ++row) {
        Cost[row] = new unsigned long*[width];
        for (int col = 0; col < width; ++col) {
            Cost[row][col] = new unsigned long[NumofDisparity]();
        }
    }
    Mat dispCensus, dispSGBM;
    CensusMatch(leftgray, rightgray, dispCensus, Cost);
    stereoSGBM(Cost, dispSGBM);

    disp = dispCensus;

}

void Stereo::CensusMatch(Mat &left, Mat &right, Mat &disp, unsigned long ***Cost) {
    int w = this->CensusWindowSize;
    int MaxDisp = this->NumofDisparity;
    width = left.cols, height = left.rows;

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
                Cost[i][j][d] = dist;
            }

        }
    }

    // Find Disparity
    for (int row = 0; row < height; row++){
        for (int col = 0; col < width; col++){
            long minCost = Cost[row][col][0];
            int minDisp = 0;
            for(int d = MaxDisp - 1; d >= 0; d--){
                if (Cost[row][col][d] < minCost){
                    minCost = Cost[row][col][d];
                    minDisp = d;
                }
            }
            disp.at<uchar>(row, col) = minDisp*255.0/MaxDisp;
        }
    }

    medianBlur(disp, disp, 3);
    imwrite("CensusMatch.jpg", disp);

    if (Visualize){
        cv::Mat disp_color;
        cv::applyColorMap(disp, disp_color, cv::COLORMAP_JET);

        cv::imshow("Disparity", disp_color);
        cv::waitKey(0);
    }
}

void Stereo::stereoSGBM(unsigned long ***Cost, Mat &disp1) {

    cout << "Starting SGBM ..." << endl;
    disp1.create(height, width, CV_8U);

    int MaxDisp = NumofDisparity;
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

    if (Visualize){
        cv::Mat disp_color;
        cv::applyColorMap(disp1, disp_color, cv::COLORMAP_JET);

        cv::imshow("Disparity", disp_color);
        cv::waitKey(0);
    }

    delete []Lr;
    delete []minLr;
    delete []S;
}
