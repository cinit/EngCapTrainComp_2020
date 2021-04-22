//
// Created by kinit on 2021-04-21.
//

#include "ClassificationManager.h"
#include "cpp_sockets/include/client.hpp"

socket_communication::Client sClient;

bool ClassificationManager::connect() {
//    sClient.Init("192.168.43.242", 11451);
    sClient.Init("127.0.0.1", 11451);
    return true;
}

ClassificationManager::Prediction ClassificationManager::predict(const cv::Mat &img) {
    if (img.cols != 96 || img.rows != 96) {
        return {};
    }
    sClient.SendImage(img);
    std::string res = sClient.Receive();
    const char *ret = res.c_str();
    ClassificationManager::Prediction pred;
    memcpy(&pred, ret, 12);
//    printf("%f,%f,%f", pred.nothing, pred.round, pred.square);
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