//
// Created by jerry on 12/24/18.
//

#ifndef STEREO_STEREOMATCH_H
#define STEREO_STEREOMATCH_H

#include <string>
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

class StereoMatch {
public:
    virtual string getKindName() = 0;
    virtual void stereoMatch(Mat &left, Mat& right, Mat &disp) = 0;

};
// StereoMatch for OpenCV function
class StereoOpenCV : public StereoMatch {
public:
    StereoOpenCV();
    string getKindName();
    void setSADWindowSize(int value);
    void setNumofDisparity(int value);
    void stereoMatch(Mat &left, Mat& right, Mat &disp);
private:
    int SADWindowSize;
    int NumofDisparity;
};

// self implemented SGBM
class Stereo : public StereoMatch {
public:
    Stereo();
    string getKindName();
    void setCensusWindowSize(int value);
    void setNumofDisparity(int value);
    void setP(int Param1, int Param2);
    void setVisualize(bool value);
    void stereoMatch(Mat &left, Mat &right, Mat &disp);
    void CensusMatch(Mat &left, Mat &right, Mat &disp, unsigned long ***Cost);
    void stereoSGBM(unsigned long ***Cost, Mat &disp1);


private:
    int CensusWindowSize;
    int NumofDisparity;
    int P1;
    int P2;
    bool Visualize;

    int height;
    int width;

};
#endif //STEREO_STEREOMATCH_H
