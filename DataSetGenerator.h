//
// Created by kinit on 2021-04-20.
//

#ifndef ENGINEERTRAINUPPERCTL_DATASETGENERATOR_H
#define ENGINEERTRAINUPPERCTL_DATASETGENERATOR_H

#include "opencv2/opencv.hpp"

void dsgResizeAndSaveImg(const cv::Mat &src, const cv::Rect &rect, const char *name);

cv::Mat dsgResizeTo96(const cv::Mat &src, const cv::Rect &rect);

#endif //ENGINEERTRAINUPPERCTL_DATASETGENERATOR_H
