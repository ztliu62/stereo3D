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
    virtual int getParamCount() = 0;
    virtual void setParamValue(int index, int value) = 0;
    virtual void stereomatch(Mat &left, Mat &right, Mat &disp) = 0;
};

class StereoOpenCV : public StereoMatch {
public:
    StereoOpenCV();
    string getKindName();
    int getParamCount();
    void setParamValue(int index, int value);
    void stereomatch(Mat &left, Mat& right, Mat &disp);
private:
    int paramCount;
    int params[3];
};

class Stereo : public StereoMatch {
public:
    Stereo();
    string getKindName();
    int getParamCount();
    void setParamValue(int index, int value);
    void stereomatch(Mat &left, Mat &right, Mat &disp);
private:
    int paramCount;
    int params[3];

};
#endif //STEREO_STEREOMATCH_H
