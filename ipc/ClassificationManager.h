//
// Created by kinit on 2021-04-21.
//

#ifndef ENGINEERTRAINUPPERCTL_CLASSIFICATIONMANAGER_H
#define ENGINEERTRAINUPPERCTL_CLASSIFICATIONMANAGER_H

#include "opencv2/opencv.hpp"

class ClassificationManager {
public:

    static const int TYPE_NOTHING = 0;
    static const int TYPE_ROUND = 1;
    static const int TYPE_SQUARE = 2;

    typedef struct {
        float nothing;
        float round;
        float square;
    } Prediction;

    typedef struct {
        int type;
        float confidence;
    } Result;

    static bool connect();

    static Prediction predict(const cv::Mat &img);

    static Result classify(const cv::Mat &img);
};

#endif //ENGINEERTRAINUPPERCTL_CLASSIFICATIONMANAGER_H
