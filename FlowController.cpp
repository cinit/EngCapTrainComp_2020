//
// Created by kinit on 2021/3/19.
//

#include "FlowController.h"

#include <iostream>
#include <opencv2/imgcodecs.hpp>
#include "opencv2/photo.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "thread"

#include "ui/Widgets.h"
#include "ui/VgaFont.h"
#include "binder/AuvManager.h"
#include "util/common.h"

#include "iostream"

#define WINDOW_NAME "EngTrainUpperCtl"

using namespace std;
using namespace cv;

bool SHOW_WINDOW = false;

typedef struct {
    bool isTurningRight;
} RunningStatus;

enum FindTubeResult {
    NOT_FOUND,
    GO_STRAIGHT,
    DRIFT_LEFT,
    DRIFT_RIGHT,
    TRANSLATE_LEFT,
    TRANSLATE_RIGHT,
    TURN_RIGHT,
};


Mat handleFrameAndSendCmdLoop(const Mat &src, AuvManager &auv, RunningStatus &status);

FindTubeResult findTube(const Mat &src, AuvManager &auv, RunningStatus &status, Mat &debug, vector<String> &dbg);

bool isPointOnDownBorder(const Point &p, const Mat &mat) {
    int height = mat.rows;
    return p.y > height - 5;
}

bool isPointOnLrupBorder(const Point &p, const Mat &mat) {
    int width = mat.cols;
    return p.y < 5 || p.x < 10 || p.x > width - 10;
}

void findTubeAndAbsorbateLoop(cv::VideoCapture &video, AuvManager &auv, bool showWindow) {
    SHOW_WINDOW = showWindow;
    Mat rawFrame;
    RunningStatus status = {};
    int frameCounter = 0;
    char text[64];
    while (true) {
        video >> rawFrame;
        if (rawFrame.empty()) {
            break;
        }
        frameCounter++;
        Mat debug = handleFrameAndSendCmdLoop(rawFrame, auv, status);
        DrawTextLeftCenterAutoColor(debug, (sprintf(text, "Frame: %d", frameCounter), text), debug.rows - 48, 16);
        if (SHOW_WINDOW) {
            imshow(WINDOW_NAME, debug);
            waitKey(100);
        } else {
            msleep(100);
        }
    }
}

Mat handleFrameAndSendCmdLoop(const Mat &src, AuvManager &auv, RunningStatus &status) {
    Mat tmp1;
    Mat debug = src.clone();
    vector<String> dbg;
    GaussianBlur(src, tmp1, Size(5, 5), 3);
    FindTubeResult tube = findTube(src, auv, status, debug, dbg);
    if (status.isTurningRight) {
        if (tube != TURN_RIGHT && tube != NOT_FOUND) {
            status.isTurningRight = false;
            auv.goStraight();
        }
    } else {
        if (tube == TURN_RIGHT) {
            status.isTurningRight = true;
            auv.goStraight();
            if (SHOW_WINDOW) {
                imshow(WINDOW_NAME, debug);
                waitKey(1500);
            } else {
                msleep(1500);
            }
            auv.turnRight();
        } else {
            switch (tube) {
                case GO_STRAIGHT: {
                    auv.goStraight();
                    break;
                }
                case DRIFT_LEFT: {
                    auv.deflectLeft();
                    break;
                }
                case DRIFT_RIGHT: {
                    auv.deflectRight();
                    break;
                }
                case TRANSLATE_LEFT: {
                    auv.translateLeft();
                    break;
                }
                case TRANSLATE_RIGHT: {
                    auv.translateRight();
                    break;
                }
            }
        }
    }
    if (SHOW_WINDOW) {
        int startY = 16;
        for (const string &str:dbg) {
            DrawTextLeftCenterAutoColor(debug, str.c_str(), 8, startY);
            startY += 16;
        }
        imshow(WINDOW_NAME, debug);
        waitKey(30);
    } else {
        msleep(30);
    }
    return debug;
}

void drawPath(Mat &out, const vector<Point> &path, const Scalar &color, int thickness = 1, int lineType = LINE_8) {
    if (path.empty()) {
        return;
    }
    Point last = path[0];
    for (const Point &p:path) {
        line(out, last, p, color, thickness, lineType);
        last = p;
    }
}

vector<Point> findOutlineUpward(Point &startPoint, const vector<Point> &doubledContour, const Mat &src) {
    vector<Point> result;
    {
        bool foundStart = false;
        bool copyingPoint = false;
        Point lastPoint(-1, -1);
        for (const Point &p:doubledContour) {
            if (!foundStart) {
                if (p == startPoint) {
                    foundStart = true;
                    lastPoint = p;
                }
            } else {
                if (!copyingPoint) {
                    if (p.y < lastPoint.y) {
                        copyingPoint = true;
                        result.emplace_back(lastPoint);
                        result.emplace_back(p);
                    } else {
                        break;
                    }
                } else {
                    result.emplace_back(p);
                    if (isPointOnLrupBorder(p, src)) {
                        break;
                    }
                }
            }
        }
    }
    if (!result.empty()) {
        return result;
    }
    {
        bool foundStart = false;
        bool copyingPoint = false;
        Point lastPoint(-1, -1);
        for (auto it = doubledContour.rbegin(); it != doubledContour.rend(); it++) {
            const Point &p = *it;
            if (!foundStart) {
                if (p == startPoint) {
                    foundStart = true;
                    lastPoint = p;
                }
            } else {
                if (!copyingPoint) {
                    if (p.y < lastPoint.y) {
                        copyingPoint = true;
                        result.emplace_back(lastPoint);
                        result.emplace_back(p);
                    } else {
                        break;
                    }
                } else {
                    result.emplace_back(p);
                    if (isPointOnLrupBorder(p, src)) {
                        break;
                    }
                }
            }
        }
    }
    return result;
}

FindTubeResult findTube(const Mat &src, AuvManager &auv, RunningStatus &status, Mat &debug, vector<String> &dbg) {
    char buf[64];
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchay;
    Mat pipeTh;
    inRange(src, Scalar(110, 100, 100), Scalar(255, 255, 255), pipeTh);
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    //开闭操作，去除噪点
    morphologyEx(pipeTh, pipeTh, MORPH_OPEN, element);
    morphologyEx(pipeTh, pipeTh, MORPH_CLOSE, element);
    findContours(pipeTh, contours, hierarchay, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    int tubeContourIdx = -1;
    //找到ROI中最大轮廓
    int currentCntLen = 0;
    for (int i = 0; i < contours.size(); i++) {
        int len = (int) arcLength(contours[i], true);
        if (len > currentCntLen) {
            currentCntLen = len;
            tubeContourIdx = i;
        }
    }
//    drawContours(debug, contours, tubeContourIdx, Scalar(0, 0, 255), 1, 8);
    if (contours.empty()) {
        dbg.emplace_back("contours.empty(): NO CONTOUR FOUND");
        return NOT_FOUND;
    }
    vector<Point> pipeContour = contours[tubeContourIdx];
    std::vector<Point> tubeApprox;
    float epsilon = (float) (0.005 * arcLength(pipeContour, true));
    std::vector<Point> pipeApprox;
    approxPolyDP(pipeContour, pipeApprox, epsilon, true);
//    drawContours(debug, vector<vector<Point>>{pipeApprox}, -1, Scalar(255, 255, 255), 1, 8);
    Point p1(-1, -1), p2(-1, -1);
    for (Point &p:pipeApprox) {
        if (isPointOnDownBorder(p, src)) {
            if (p1.x < 0 || p.x < p1.x) {
                p1 = p;
            }
            if (p2.x < 0 || p.x > p2.x) {
                p2 = p;
            }
        }
    }
    sprintf(buf, "P1=(%d,%d) P2=(%d,%d)", p1.x, p1.y, p2.x, p2.y);
    if (p1.x < 0 || p2.x < 0) {
        dbg.emplace_back("BASE POINTS NOT FOUND");
    } else {
        dbg.emplace_back(buf);
    }
    vector<Point> outlineLeft, outlineRight;
    {
        vector<Point> dextro;
        dextro.insert(dextro.end(), pipeApprox.begin(), pipeApprox.end());
        dextro.insert(dextro.end(), pipeApprox.begin(), pipeApprox.end());
        outlineLeft = findOutlineUpward(p1, dextro, src);
        outlineRight = findOutlineUpward(p2, dextro, src);
    }
    if (outlineLeft.empty() || outlineRight.empty()) {
        !outlineLeft.empty() || (dbg.emplace_back("Outline left empty"), false);
        !outlineRight.empty() || (dbg.emplace_back("Outline right empty"), false);
    } else {
        drawPath(debug, outlineLeft, Scalar(255, 255, 255), 2, 8);
        drawPath(debug, outlineRight, Scalar(255, 255, 255), 2, 8);
        for (int i = 0; i < outlineLeft.size(); ++i) {
            const Point &p = outlineLeft[i];
            DrawTextLeftCenterAutoColor(debug, (sprintf(buf, "L%d", i), buf), p.x, p.y);
        }
        for (int i = 0; i < outlineRight.size(); ++i) {
            const Point &p = outlineRight[i];
            DrawTextLeftCenterAutoColor(debug, (sprintf(buf, "R%d", i), buf), p.x, p.y);
        }
        //start find corner by outlineRight
        Vec2f sum;


    }
    return NOT_FOUND;
}

