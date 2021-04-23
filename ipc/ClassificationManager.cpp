//
// Created by kinit on 2021-04-21.
//

#include "ClassificationManager.h"
#include "cpp_sockets/include/client.hpp"

#include "opencv2/opencv.hpp"
#include "algorithm"

socket_communication::Client sClient;

using namespace cv;
using namespace std;

bool ClassificationManager::connect() {
//    sClient.Init("192.168.43.242", 11451);
//    sClient.Init("127.0.0.1", 11451);
    return true;
}

ClassificationManager::Prediction ClassificationManager::predict(const cv::Mat &img) {
    if (img.cols != 96 || img.rows != 96) {
        return {1, 0, 0};
    }
    Mat grey;
    cvtColor(img, grey, COLOR_BGR2GRAY);
    Mat bin;
    threshold(grey, bin, 90, 255, THRESH_BINARY);
//    imshow("bin", bin);
//    imshow("grey", grey);
    if (mean(bin(Rect(40, 40, 16, 16)))[0] > 13) {
        return {1, 0, 0};
    }
    int validCorners = 0;
    {
        if (mean(bin(Rect(0, 0, 4, 4)))[0] > 253) {
            validCorners++;
        }
        if (mean(bin(Rect(0, 91, 4, 4)))[0] > 253) {
            validCorners++;
        }
        if (mean(bin(Rect(91, 0, 4, 4)))[0] > 253) {
            validCorners++;
        }
        if (mean(bin(Rect(91, 91, 4, 4)))[0] > 253) {
            validCorners++;
        }
        if (validCorners < 3) {
            return {1, 0, 0};
        }
    }
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat bin_close;
    morphologyEx(bin, bin_close, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(9, 9)));
    findContours(bin_close, contours, hierarchy, RETR_LIST, CHAIN_APPROX_SIMPLE);
    vector<Point> outline = [&contours]() {
        float maxArea = 0;
        int maxIdx = -1;
        for (int i = 0; i < contours.size(); i++) {
            auto &cnt = contours[i];
            float area = contourArea(cnt);
            if (area > maxArea && area < 96 * 96 * 0.8) {
                maxArea = area;
                maxIdx = i;
            }
        }
        return contours[maxIdx];
    }();
    vector<Point> cnt;
    convexHull(outline, cnt);
    float epsilon = (float) (0.007f * arcLength(cnt, true));
    std::vector<Point> cntApprox;
    approxPolyDP(cnt, cntApprox, epsilon, true);
    drawContours(img, vector<vector<Point>>{cntApprox}, -1, Scalar(0, 255, 0));
    int count = cntApprox.size();
//    cout << count << ",";
    if (count <= 11) {
        return {0, 0, 1};
    } else {
        return {0, 1, 0};
    }
//
//    sClient.SendImage(img);
//    std::string res = sClient.Receive();
//    const char *ret = res.c_str();
    float a = (rand() % 10) / 10.0;
    ClassificationManager::Prediction pred{0, a, 1.0f - a};

    return pred;
}

ClassificationManager::Result ClassificationManager::classify(const cv::Mat &img) {
    ClassificationManager::Prediction pred = predict(img);
    if (pred.square > pred.round && pred.square > pred.nothing) {
        return {TYPE_SQUARE, pred.square};
    } else if (pred.round > pred.square && pred.round > pred.nothing) {
        return {TYPE_ROUND, pred.round};
    } else {
        return {TYPE_NOTHING, pred.nothing};
    }
}

const char *ClassificationManager::getLabelName(int type) {
    const char *name;
    switch (type) {
        case ClassificationManager::TYPE_ROUND:
            name = "round";
            break;
        case ClassificationManager::TYPE_SQUARE:
            name = "square";
            break;
        default:
            name = "nothing";
    }
    return name;
}