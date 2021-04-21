//
// Created by kinit on 2021-04-20.
//

#include "DataSetGenerator.h"
#include "unistd.h"
#include "util/common.h"

using namespace cv;

void dsgResizeAndSaveImg(const cv::Mat &src, const cv::Rect &rect, const char *name) {
    Mat dst96u3;
    cv::resize(src(rect), dst96u3, cv::Size(96, 96), 0, 0, INTER_LINEAR);
    char path[128];
    uint64_t time = currentTimeMillis();
    sprintf(path, "/home/kinit/CLionProjects/Abs_96x96/%s/%ld.jpg", name, time);
    imwrite(path, dst96u3);
}

Mat dsgResizeTo96(const cv::Mat &src, const cv::Rect &rect) {
    Mat dst96u3;
    cv::resize(src(rect), dst96u3, cv::Size(96, 96), 0, 0, INTER_LINEAR);
    return dst96u3;
}