//
// Created by kinit on 2021/3/19.
//

#include "FlowController.h"

#include <iostream>
#include <opencv2/opencv.hpp>
#include "opencv2/photo.hpp"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "thread"

#include "ui/Widgets.h"
#include "ui/VgaFont.h"
#include "binder/AuvManager.h"
#include "util/common.h"
#include "random"
#include "DataSetGenerator.h"
#include "ipc/ClassificationManager.h"
#include "iostream"

#define WINDOW_NAME "EngTrainUpperCtl"

#define DEBUG_FF 100

using namespace std;
using namespace cv;

bool SHOW_WINDOW = false;

uint64_t gLastBatVolTime = 0;
float gLastBatVolValue = 0;

typedef struct {
//    bool isTurningRight;
//    int cornerCounter;
    bool hasAbsorbateNow;
} RunningStatus;

typedef struct {
    bool found;
    float deltaX;
    float deltaYaw;
} TubeDetectionError;

typedef struct {
    int imgWidth;
    int imgHeight;
    bool hasTube;
    int tubeBottomX;
    // left- right+
    float tubeDirectionRad;
    // dx/dy, gt 0, usually 0.x
    float tubeParallelPerspectiveSlope;
    bool hasCorner;
    Point cornerPosition;
} TubeDetectResult;

Mat handleFrameAndSendCmdLoop(const Mat &src, AuvManager &auv, RunningStatus &status);

TubeDetectResult findTube(const Mat &src, AuvManager &auv, RunningStatus &status, Mat &debug, vector<String> &dbg);

vector<Point> getSamplePoints(const Mat &src, Mat &debug, vector<String> &dbg, int roi_all);

// 核心代码, 求斜率w,截距b
bool fit(const vector<Point> &points, Vec2f &out) {
    int m = points.size();
    if (m < 3) {
        return false;
    }
    float x_bar = [&points] {
        float sum = 0;
        for (auto &p:points) {
            sum += float(p.y);
        }
        return sum / float(points.size());
    }();
    int sum_yx = 0;
    int sum_x2 = 0;
    int sum_delta = 0;
    for (int i = 0; i < m; i++) {
        int y = points[i].x;
        int x = points[i].y;
        sum_yx += y * (x - x_bar);
        sum_x2 += pow(x, 2);
    }
    // 根据公式计算w
    float w = sum_yx / (sum_x2 - m * pow(x_bar, 2));
    for (int i = 0; i < m; i++) {
        int y = points[i].x;
        int x = points[i].y;
        sum_delta += (y - w * x);
    }
    float b = sum_delta / m;
    out = Vec2f(w, b);
    return true;
}

TubeDetectionError findTubeLsm(const Mat &src, AuvManager &auv, RunningStatus &status,
                               Mat &debug, vector<String> &dbg, float *pwb) {
    vector<Point> points = getSamplePoints(src, debug, dbg, 20);
    Vec2f wb;
    if (fit(points, wb)) {
        float w = wb[0];
        float b = wb[1];
        if (pwb != nullptr) {
            pwb[0] = w;
            pwb[1] = b;
        }
        line(debug, Point(b, 0), Point(int(float(src.rows) * w + b), src.rows), Scalar(0, 255, 0), 2);
        float deltaX = int(float(src.rows / 2) * w + b) - src.cols / 2;
        float rad = atan(w);
        //因为摄像头是斜的不是垂直朝下的,所以给角度一个简单的比例放大
        if (rad >= 0) {
            rad = pow(rad, 0.75) * 3;
        } else {
            rad = -(pow(-rad, 0.75) * 3);
        }
        return TubeDetectionError{true, deltaX, -rad};
    } else {
        return TubeDetectionError{false, 0, 0};
    }
}

bool isPointOnDownBorder(const Point &p, const Mat &mat) {
    int height = mat.rows;
    return p.y > height - 10;
}

bool isPointOnLrupBorder(const Point &p, const Mat &mat) {
    int width = mat.cols;
    return p.y < 10 || p.x < 10 || p.x > width - 10;
}

bool isPointOnRightBorder(const Point &p, const Mat &mat) {
    int width = mat.cols;
    return p.x > width - 10;
}

int g_thres_v_up = 160;

void findTubeAndAbsorbateLoop(cv::VideoCapture &video, AuvManager &auv, bool showWindow) {
    SHOW_WINDOW = showWindow;
    Mat rawFrame;
    RunningStatus status = {};
    status.hasAbsorbateNow = false;
    int frameCounter = 0;
    char text[64];
    if (showWindow) {
        startWindowThread();
        namedWindow(WINDOW_NAME);
        createTrackbar("hsv_up", WINDOW_NAME, &g_thres_v_up, 255);
    }
    while (true) {
        uint64_t currTime = currentTimeMillis();
        if (currTime - gLastBatVolTime > 1000) {
            gLastBatVolTime = currTime;
            gLastBatVolValue = auv.getBatteryVoltage();
        }
        uint64 t1 = currentTimeMillis();
        video >> rawFrame;
        if (rawFrame.empty()) {
            break;
        }
        uint64 t2 = currentTimeMillis();
        frameCounter++;
        Mat debug = handleFrameAndSendCmdLoop(rawFrame, auv, status);
        DrawTextLeftCenterAutoColor(debug, (sprintf(text, "Battery: %0.3fV", gLastBatVolValue), text),
                                    debug.rows - 48, 16);
        DrawTextLeftCenterAutoColor(debug, (sprintf(text, "Frame: %d", frameCounter), text), debug.rows - 48, 32);
        DrawTextLeftCenterAutoColor(debug, (sprintf(text,
                                                    "select cost: %dms", int(t2 - t1)), text), debug.rows - 48, 48);
        if (SHOW_WINDOW) {
            imshow(WINDOW_NAME, debug);
            int code = waitKey(1);
            if (code == 'q' || code == 'Q') {
                auv.stop();
                msleep(20);
                auv.rtlControlMotionOutput(0, 0, 0, 0);
                msleep(20);
                auv.stop();
                msleep(20);
                auv.rtlControlMotionOutput(0, 0, 0, 0);
                msleep(20);
                return;
            }
        }
        printf("%d,", frameCounter);
        cout.flush();
        msleep(DEBUG_FF);
    }
}

TubeDetectionError fallbackSeekForOperation(const Mat &src, AuvManager &auv, RunningStatus &status,
                                            Mat &debug, vector<String> &dbg) {
    char buf[64];
    float sampleRelativeY = 0.3;
    int samplePointTotal = 9;
    int delta = src.cols / (samplePointTotal + 1);
    int values[samplePointTotal];
    int posY = int(sampleRelativeY * float(src.rows));
    for (int i = 0; i < samplePointTotal; ++i) {
        char *pp;
        unsigned char r, g, b;
        pp = (char *) ((int64_t) src.data + (int64_t) src.step.buf[0] * posY +
                       (int64_t) src.step.buf[1] * (delta + delta * i));
        r = pp[0] & 0xFF;
        g = pp[1] & 0xFF;
        b = pp[2] & 0xFF;
        values[i] = r + g + b;
    }
    int currIdx = -1;
    int currMax = samplePointTotal / 2;
    for (int j = 0; j < samplePointTotal; ++j) {
        if (currMax < values[j]) {
            currMax = values[j];
            currIdx = j;
        }
    }
    int tubeX = delta * (currIdx + 1);
    Point detectPoint(tubeX, posY);
    rectangle(debug, detectPoint - Point(20, 20), detectPoint + Point(20, 20), Scalar(0, 0, 255), 2);
    DrawTextLeftCenterAutoColor(debug, "FALLBACK", detectPoint.x - 20, detectPoint.y + 32);
    dbg.emplace_back((sprintf(buf, "Fallback(%d,%d)", tubeX, posY), buf));
    if (currIdx < samplePointTotal / 2 - 1) {
        return TubeDetectionError{true, float(detectPoint.x - src.cols / 2), 0};
    }
    if (currIdx > samplePointTotal / 2 + 1) {
        return TubeDetectionError{true, float(detectPoint.x - src.cols / 2), 0};
    }
    return TubeDetectionError{false, 0, 0};
}

vector<Rect> findAbsorbates(const Mat &src, AuvManager &auv, RunningStatus &status, Mat &debug, vector<String> &dbg) {
    Mat rawCsv;
    char buf[64];
    cvtColor(src, rawCsv, COLOR_BGR2HSV);// 综合对比发现lab能够很快的分辨出红色圆圈
    {
        int deltaX = src.cols / 11;
        int deltaY = src.rows / 11;
        int vsum, ssum = 0;
        for (int i = 0; i < 10; i++) {
            for (int j = 0; j < 10; j++) {
                uint32_t hsv = src.at<uint32_t>(deltaY / 2 + i * deltaY, deltaX / 2 + i * deltaX);
                ssum += ((hsv >> 16) & 0xFF);
                vsum += ((hsv >> 24) & 0xFF);
            }
        }
        float savg = float(ssum) / 100.0f;
        float havg = float(vsum) / 100.0f;
        dbg.emplace_back((sprintf(buf, "Savg=%.1f", savg), buf));
        dbg.emplace_back((sprintf(buf, "Vavg=%.1f", havg), buf));
    }
    Mat region;
    inRange(rawCsv, Scalar(0, 0, 0), Scalar(255, 255, g_thres_v_up), region);
    erode(region, region, getStructuringElement(MORPH_RECT, Size(3, 3)));
    dilate(region, region, getStructuringElement(MORPH_RECT, Size(5, 5)));
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    findContours(region, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    drawContours(debug, contours, -1, Scalar(0, 255, 0), 1);
    std::random_device rd;
    vector<Rect> results;
    int maxSingleLen = int(0.8 * min(src.cols, src.rows));
    for (auto &cnt:contours) {
        Rect rect = boundingRect(cnt);
        float alen = float(rect.height + rect.width) / 2.0f;
        float area = contourArea(cnt);
        float duty = area / (alen * alen);
        if (duty < 0.5 || alen * 1.3 > maxSingleLen || alen < 20) {
            continue;
        }
        Scalar color(rd() & 0xFF, rd() & 0xFF, rd() & 0xFF);
        rectangle(debug, rect, color, 2);
//        DrawTextLeftCenter(debug, (sprintf(buf, "%.3f", duty), buf), rect.x, rect.y - 10, color);
        results.emplace_back(rect);
    }
    return results;
}

Rect getContainingSquareRect(const Rect &target, const Rect &container) {
    if (target.width == target.height) {
        return target;
    } else if (target.width > target.height) {
        //fat
        int delta = abs(target.height - target.width);
        int len = target.width;
        int da = delta / 2;
        int db = delta - da;
        int start = target.y - da;
        if (start + len > container.height) {
            start = container.height - len;
        }
        if (start < 0) {
            start = 0;
        }
        return {target.x, start, len, len};
    } else {
        //tall
        int delta = abs(target.height - target.width);
        int len = target.height;
        int da = delta / 2;
        int db = delta - da;
        int start = target.x - da;
        if (start + len > container.width) {
            start = container.width - len;
        }
        if (start < 0) {
            start = 0;
        }
        return {start, target.y, len, len};
    }
}

Rect inflateRectBy(const Rect &target, const Rect &container, float ratio) {
    ratio /= 2.0;
    int dx = int(float(target.width) * ratio);
    int dy = int(float(target.height) * ratio);
    int startx = max(target.x - dx, 0);
    int starty = max(target.y - dy, 0);
    int endx = min(target.x + target.width + dx, container.width);
    int endy = min(target.y + target.height + dy, container.height);
    return {startx, starty, endx - startx, endy - starty};
}

float lastMotionCmd[4] = {0};

float getRectCenterToLineDistance(const Rect &rect, float w, float b) {
    int x = rect.x + rect.width / 2;
    int y = rect.y + rect.height / 2;
    // x=wy+b,
    float dist = abs(w * y - x + b) / sqrt(w * w + 1);
    return dist;
}

Mat handleFrameAndSendCmdLoop(const Mat &src, AuvManager &auv, RunningStatus &status) {
    Mat tmp1;
    Mat debug = src.clone();
    char buf[64];
    vector<String> dbg;
    dbg.emplace_back((sprintf(buf, "%dx%d", src.cols, src.rows), buf));
    GaussianBlur(src, tmp1, Size(5, 5), 3);
    TubeDetectResult tube = findTube(src, auv, status, debug, dbg);
//    {
//        // draw dbg
//        if (tube.hasCorner) {
//            if(tube.)
//        }
//    }
    TubeDetectionError tubeError = {false, 0, 0};
    float extraWb[2];
    if (false) {
        {
            if (tube.hasTube) {
                float deg = 57.3f * tube.tubeDirectionRad;
                dbg.emplace_back((sprintf(buf, "Tdeg: %+0.1f", deg), buf));
                // [-1.0f,1.0f]
                float relPosX = float(2.0f * (float) tube.tubeBottomX / (float) tube.imgWidth) - 1.0f;
                dbg.emplace_back((sprintf(buf, "Xrel: %+0.2f", relPosX), buf));
                tubeError.found = true;
                tubeError.deltaX = float(tube.tubeBottomX - src.cols / 2);
                tubeError.deltaYaw = tube.tubeDirectionRad * 57.3f;
            }
        }
        if (!tubeError.found) {
            tubeError = fallbackSeekForOperation(src, auv, status, debug, dbg);
        }
    } else {
        tubeError = findTubeLsm(src, auv, status, debug, dbg, extraWb);
    }
    if (tubeError.found) {
        float refs[4];
        auv.updateCurrentError(tubeError.deltaX, tubeError.deltaYaw * 57.3f, &refs);
        for (int k = 0; k < 4; k++) {
            lastMotionCmd[k] = refs[k];
        }
        dbg.emplace_back((sprintf(buf, "  Xo: %+.1f", refs[0]), buf));
        dbg.emplace_back((sprintf(buf, "  Yo: %+.1f", refs[1]), buf));
        dbg.emplace_back((sprintf(buf, "  Zo: %+.1f", refs[2]), buf));
        dbg.emplace_back((sprintf(buf, "  Wo: %+.1f", refs[3]), buf));
    } else {
        int action = 0;
        if (lastMotionCmd[1] > 0) {
            action = 1;
        } else if (lastMotionCmd[1] < 0) {
            action = -1;
        }
        int delY = 0;
        int delW = 0;
        if (action == 1) {
            dbg.emplace_back("**RESCUE MODE** go right");
            delY = 35;
            delW = 42;
        } else if (action == -1) {
            dbg.emplace_back("**RESCUE MODE** go left");
            delY = -35;
            delW = -42;
        }
        auv.rtlControlMotionOutput(0, delY, 0, delW);
        dbg.emplace_back((sprintf(buf, "  Yo: %d", delY), buf));
        dbg.emplace_back((sprintf(buf, "  Wo: %d", delW), buf));
    }
    {
        vector<Rect> absorbates = findAbsorbates(src, auv, status, debug, dbg);
        vector<Rect> rectAbs;
        Rect full = Rect(0, 0, src.cols, src.rows);
        int t_thisHas = 0;
        int minDistIdx = -1;
        float minDist = -1;
        ClassificationManager::Result bestResult;
        for (int i = 0; i < absorbates.size(); i++) {
            auto &ra = absorbates[i];
            Rect r = getContainingSquareRect(inflateRectBy(ra, full, 0.1f), full);
            rectAbs.emplace_back(r);
            rectangle(debug, r, Scalar(0, 0, 255));
            ClassificationManager::Result result = ClassificationManager::classify(dsgResizeTo96(src, r));
            const char *name;
            switch (result.type) {
                case ClassificationManager::TYPE_ROUND:
                    name = "round";
                    break;
                case ClassificationManager::TYPE_SQUARE:
                    name = "square";
                    break;
                default:
                    name = "nothing";
            }
            if (result.type != ClassificationManager::TYPE_NOTHING) {
                if (tubeError.found) {
                    float dist = getRectCenterToLineDistance(ra, extraWb[0], extraWb[1]);
                    if (minDist < 0 || dist < minDist) {
                        minDist = dist;
                        minDistIdx = i;
                        bestResult = result;
                    }
                    t_thisHas++;
                }
                DrawTextLeftCenterAutoColor(debug, (sprintf(buf, "%s %.3f", name, result.confidence), buf),
                                            r.x, r.y - 10);
            }

//            ClassificationManager::predict(dsgResizeTo96(src, r));
//            dsgResizeAndSaveImg(src, r, "test");
//            msleep(20);
        }
        if (tubeError.found) {
            bool lastHasAbs = status.hasAbsorbateNow;
            bool thisHasAbs = status.hasAbsorbateNow = !!t_thisHas;
            dbg.emplace_back((sprintf(buf, "AbsCnt=%d", t_thisHas), buf));
            if (lastHasAbs == false && thisHasAbs == true) {
                auv.reportAdsorbate(1, bestResult.type == ClassificationManager::TYPE_SQUARE ?
                                       AuvManager::SHAPE_RECTANGLE : AuvManager::SHAPE_CIRCLE);
            }
        }
    }
    if (SHOW_WINDOW) {
        DrawTextLeftCenterAutoColor(debug, "+-----------+ 100px", 16, debug.rows - 32);
        int startY = 16;
        for (const string &str:dbg) {
            DrawTextLeftCenterAutoColor(debug, str.c_str(), 8, startY);
            startY += 16;
        }
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
                        if (isPointOnLrupBorder(p, src)) {
                            break;
                        }
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
                        if (isPointOnLrupBorder(p, src)) {
                            break;
                        }
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

inline float diffRad180(float a, float b) {
    float diff = abs(a - b);
    while (diff >= CV_PI) {
        diff -= CV_PI;
    }
    return diff;
}

/**
 *
 * @param test
 * @param reference
 * @param errorThreshold
 * @return true if ok, false if corner
 */
bool checkVectorConsistency(const Vec2i &test, const Vec2i &reference, int errorThreshold) {
    double radRef = atan2(reference[1], reference[0]);
    double testRef = atan2(test[1], test[0]);
    double diffRad = diffRad180(radRef, testRef);
    double diffDeg = diffRad * 57.3;
    if (diffDeg < 10.0) {
        return true;
    }
    if (diffDeg > 60.0) {
        return false;
    }
    float len = hypot(reference[0], reference[1]);
    if (sin(diffRad) * len > errorThreshold) {
        return false;
    }
    return true;
}

Vec2f basify(const Vec2f &v) {
    if (v[0] == 0 && v[1] == 0) {
        return Vec2f(0, 0);
    }
    float val = 1.0f / hypot(v[0], v[1]);
    return Vec2f(v[0] * val, v[1] * val);
}

Vec2f basify(const Vec2i &v) {
    return basify(Vec2f(v[0], v[1]));
}

void find_blob(int roi_i, int roi_num, const Mat &binary,
               vector<vector<Point>> &contours, Rect &roi, Mat *debug = nullptr) {
    int width = binary.cols;
    int height = binary.rows;
    roi = Rect(0, roi_i * height / roi_num, width, height / roi_num);
    Mat roi_img_binary = binary(roi);
    contours.clear();
    findContours(roi_img_binary, contours, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
    if (debug != nullptr) {
        drawContours((*debug)(roi), contours, -1, Scalar(0, 0, 0), 1);
    }
}

vector<Point> getSamplePoints(const Mat &src, Mat &debug, vector<String> &dbg, int roi_all) {
    vector<Point> points;
    Mat lab,close;
//    cvtColor(src, lab, COLOR_BGR2Lab);// 综合对比发现lab能够很快的分辨出红色圆圈
//    Mat lab_frame;
//    inRange(lab, Scalar(182, 0, 108), Scalar(255, 255, 255), lab_frame);
//    Mat close;
//    morphologyEx(lab_frame, close, MORPH_CLOSE, getStructuringElement(MORPH_ELLIPSE, Size(13, 13)));
//
    inRange(src, Scalar(180, 180, 150), Scalar(255, 255, 255), lab);
    Mat tmpStructEle = getStructuringElement(MORPH_RECT, Size(5, 5));
    //开闭操作，去除噪点
    morphologyEx(lab, lab, MORPH_OPEN, tmpStructEle);
    morphologyEx(lab, close, MORPH_CLOSE, tmpStructEle);


    for (int i = 0; i < roi_all; i++) {
        vector<vector<Point>> contour;
        Rect roi;
        find_blob(i, roi_all, close, contour, roi, &debug);
        if (contour.size() == 1) {
            // 通过blob面积与矩形和三角形的面积进行比较来过滤
            Rect r = boundingRect(contour[0]);
            int area_rect = r.width * r.height;
            int area_half = area_rect / 2;
            double area = contourArea(contour[0]);
            if (abs(area_rect - area) < abs(area - area_half)) {
                pass;
            } else {
                continue;
            }
            // 通过判断blob的宽度来过滤
            if (r.width > src.cols * 0.8) {
                continue;
            }
            Moments M = moments(contour[0]);
            if (M.m00 != 0) {
                int cX = int(M.m10 / M.m00);
                int cY = int(M.m01 / M.m00);
                Point p = Point(cX, cY) + roi.tl();
                circle(debug, p, 2, Scalar(255, 0, 0), -1);
                points.emplace_back(p);
            } else {
                pass;
            }
        }
    }
    return points;
}

vector<Point> followStraightDown(const vector<Point> &outline, const Point &start, Nullable Point *lastPoint) {
    Vec2i sumVec(0, 0);
    vector<Point> path;
    int count = 0;
    int cornerThreshold = 40;
    Point currentPoint;
    for (const Point &p:outline) {
        Vec2i det(p.x - start.x, p.y - start.y);
        if (count < 2) {
            sumVec += det;
            path.emplace_back(p);
        } else {
            if (hypot(sumVec[0], sumVec[1]) < 2 * cornerThreshold
                || checkVectorConsistency(det, sumVec, cornerThreshold)) {
                // is line
                sumVec += det;
                path.emplace_back(p);
            } else {
                // corner
                break;
            }
        }
        currentPoint = p;
        count++;
    }
    Vec2f direction = basify(sumVec);
    Vec2i vecP2ActualEnd = Vec2i(currentPoint.x - start.x, currentPoint.y - start.y);
    if (lastPoint != nullptr) {
        *lastPoint = currentPoint;
    }
    return path;
}

void bidirectionalExclude(vector<Point> &inOutA, vector<Point> &inOutB) {
    vector<Point> tmpA = inOutA, tmpB = inOutB;
    for (auto currP = inOutA.begin(); currP != inOutA.end(); ++currP) {
        if (*std::find(tmpB.begin(), tmpB.end(), *currP) != *tmpB.end()) {
            inOutA.erase(currP);
            currP--;
        }
    }
    for (auto currP = inOutB.begin(); currP != inOutB.end(); ++currP) {
        if (*std::find(tmpA.begin(), tmpA.end(), *currP) != *tmpA.end()) {
            inOutB.erase(currP);
            currP--;
        }
    }
}

float countErr(const vector<Point> &points, int index) {
    Point avg(0, 0);
    for (int i = 0; i < points.size(); ++i) {
        if (i == index) {
            continue;
        }
        avg += points[i];
    }
    avg.x /= points.size() - ((index < 0) ? 0 : 1);
    avg.y /= points.size() - ((index < 0) ? 0 : 1);
    float err = 0;
    for (int i = 0; i < points.size(); ++i) {
        if (i == index) {
            continue;
        }
        err = hypot(avg.x - points[i].x, avg.y - points[i].y);
    }
    return err;
}

Point average4(const Point &p1, const Point &p2, const Point &p3, const Point &p4) {
    float TH_PCT = 0.5;
//    const Point pts[4] = {p1, p2, p3, p4};
//    Vec2i cnrVec = p1 + p2 + p3 + p4;
//    Point avg(cnrVec[0] / 4, cnrVec[1] / 4);
//    float errSum = 0;
//    float errs[4];
//    for (int i = 0; i < 4; ++i) {
//        errs[i] = hypot(avg.x - pts[i].x, avg.y - pts[i].y);
//        errSum += errs[i];
//    }
//    float currentMax = 0;
//    int ko = -1;
//    for (int i = 0; i < 4; ++i) {
//        float errs[0] / errSum;
//        if ()
//    }
//    if ( < TH_PCT && errs[1] / errSum < TH_PCT
//           && errs[2] / errSum < TH_PCT && errs[3] / errSum < TH_PCT) {
//        return avg;
//    } else {
//
//    }
    const vector<Point> points = vector<Point>{p1, p2, p3, p4};
    float totalError = countErr(points, -1);
    float singleError[4];
    int index = -1;
    float maxError = 0;
    for (int i = 0; i < 4; ++i) {
        singleError[i] = countErr(points, i);
        float temp = (totalError - singleError[i]) / totalError;
        if (temp >= TH_PCT) {
            if (temp > maxError) {
                maxError = temp;
                index = i;
                break;
            }
        }
    }
    Point result(0, 0);
    for (int i = 0; i < 4; ++i) {
        if (i == index) {
            continue;
        }
        result += points[i];
    }
    result.x /= points.size() - ((index < 0) ? 0 : 1);
    result.y /= points.size() - ((index < 0) ? 0 : 1);
    return result;
}


TubeDetectResult findTube(const Mat &src, AuvManager &auv, RunningStatus &status, Mat &debug, vector<String> &dbg) {
    char buf[64];
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchy;
    Mat pipeTh;
    inRange(src, Scalar(180, 180, 150), Scalar(255, 255, 255), pipeTh);
    Mat tmpStructEle = getStructuringElement(MORPH_RECT, Size(5, 5));
    //开闭操作，去除噪点
    morphologyEx(pipeTh, pipeTh, MORPH_OPEN, tmpStructEle);
    morphologyEx(pipeTh, pipeTh, MORPH_CLOSE, tmpStructEle);
    findContours(pipeTh, contours, hierarchy, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE);
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
    drawContours(debug, contours, tubeContourIdx, Scalar(0, 0, 255), 1, 8);
    if (contours.empty()) {
        dbg.emplace_back("contours.empty(): NO CONTOUR FOUND");
        return TubeDetectResult{src.cols, src.rows, false, 0, 0, 0, false};
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
        return TubeDetectResult{src.cols, src.rows, false, 0, 0, 0, false};
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
        {
            //start find corner by outlineRight
            Point lastDownRight, lastDownLeft/*, endDownRight, endDownLeft*/;
            vector<Point> rightDownPath = followStraightDown(outlineRight, p2, &lastDownRight);
            vector<Point> leftDownPath = followStraightDown(outlineLeft, p1, &lastDownLeft);
            if (outlineLeft[outlineLeft.size() - 1] != lastDownLeft
                && outlineRight[outlineRight.size() - 1] != lastDownRight
                && isPointOnRightBorder(outlineLeft[outlineLeft.size() - 1], src)
                && isPointOnRightBorder(outlineRight[outlineRight.size() - 1], src)) {
                std::vector<Point> reservedPath1 = outlineLeft, reservedPath2 = outlineRight;
                std::reverse(reservedPath1.begin(), reservedPath1.end());
                std::reverse(reservedPath2.begin(), reservedPath2.end());
                vector<Point> leftLrPath = followStraightDown(reservedPath1, reservedPath1[0], nullptr);
                vector<Point> rightLrPath = followStraightDown(reservedPath2, reservedPath2[0], nullptr);
//                bidirectionalExclude(leftDownPath, leftLrPath);

                Point p1u, p1d, p2u, p2d;
                p1u = leftLrPath[leftLrPath.size() - 1];
                p1d = leftDownPath[leftDownPath.size() - 1];
                p2u = rightLrPath[rightLrPath.size() - 1];
                p2d = rightDownPath[rightDownPath.size() - 1];

                line(debug, p1u, p2u, Scalar(0, 0, 0), 2);
                line(debug, p1d, p2d, Scalar(0, 0, 0), 2);

                DrawTextLeftCenterAutoColor(debug, "*[1-U]", p1u.x - 32, p1u.y + 16);
                DrawTextLeftCenterAutoColor(debug, "*[1-D]", p1d.x - 8, p1d.y + 16);
                DrawTextLeftCenterAutoColor(debug, "*[2-U]", p2u.x - 32, p2u.y + 16);
                DrawTextLeftCenterAutoColor(debug, "*[2-D]", p2d.x - 8, p2d.y + 16);
                Point cornerPoint = average4(p1u, p2u, p1d, p2d);
                rectangle(debug, cornerPoint - Point(20, 20), cornerPoint + Point(20, 20), Scalar(0, 0, 255), 2);
                DrawTextLeftCenterAutoColor(debug, "* CORNER", cornerPoint.x - 4, cornerPoint.y);
                dbg.emplace_back((sprintf(buf, "Corner(%d,%d)", cornerPoint.x, cornerPoint.y), buf));
                int centerBottom = (p1.x + p2.x) / 2;
                Vec2f directionVec = basify(Vec2f(cornerPoint.x - centerBottom, cornerPoint.y - src.rows));
                float rad = asin(directionVec[0]);
                return TubeDetectResult{src.cols, src.rows, true, centerBottom, rad,
                                        (float(p1d.x - p1.x) / float(p1d.y - p1.y) +
                                         float(p2d.x - p2.x) / float(p2d.y - p2.y)) / 2.0f,
                                        true, cornerPoint};
            } else {
                DrawTextLeftCenterAutoColor(debug, "*[R-LAST]", lastDownRight.x - 8, lastDownRight.y + 16);
                DrawTextLeftCenterAutoColor(debug, "*[L-LAST]", lastDownLeft.x - 8, lastDownLeft.y + 16);
                dbg.emplace_back("Corner not found");
                int centerBottom = (p1.x + p2.x) / 2;
                Vec2i tmp = lastDownRight + lastDownLeft;
                Vec2f directionVec = basify(Vec2f(tmp[0] / 2 - centerBottom, tmp[1] / 2 - src.rows));
                float rad = asin(directionVec[0]);
                return TubeDetectResult{src.cols, src.rows, true, centerBottom, rad,
                                        (float(lastDownLeft.x - p1.x) / float(lastDownLeft.y - p1.y) +
                                         float(lastDownRight.x - p2.x) / float(lastDownRight.y - p2.y)) / 2.0f, false};
            }
        }
    }
}

