//
// Created by kinit on 2021/3/19.
//

#ifndef ENGINEERTRAINUPPERCTL_FLOWCONTROLLER_H
#define ENGINEERTRAINUPPERCTL_FLOWCONTROLLER_H

#include "opencv2/opencv.hpp"
#include "binder/AuvManager.h"

void findTubeAndAbsorbateLoop(cv::VideoCapture &video, AuvManager &auv, bool showWindow);

#endif //ENGINEERTRAINUPPERCTL_FLOWCONTROLLER_H
