//
// Created by jerry on 12/24/18.
//

#include "StereoMatch.h"
#include <opencv2/calib3d/calib3d.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;

StereoOpenCV::StereoOpenCV() {
    this->paramCount = 3;
    this->params[0] = 11;
    this->params[1] = 6*16;
    this->params[2] = 0;
}

string StereoOpenCV::getKindName() {
    return "SGBM_OpenCV";
}


