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

Mat Pretreatment(const Mat &src);

Mat Lpretreatment(Mat &src);

String CR(const Mat &src1, Mat &debug);

Mat handleFrameAndSendCmdLoop(const Mat &src, AuvManager &auv, RunningStatus &status);

FindTubeResult findTube(const Mat &src, AuvManager &auv, RunningStatus &status, Mat &debug);


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
        DrawTextLeftCenterAutoColor(debug, (sprintf(text, "frame=%d", frameCounter), text), debug.rows - 48, 20);
        if (SHOW_WINDOW) {
            imshow(WINDOW_NAME, debug);
            waitKey(100);
        } else {
            msleep(100);
        }
    }
}


Mat Lpretreatment(Mat &src) {
    Mat dst;
    inRange(src, Scalar(110, 100, 100), Scalar(255, 255, 255), dst);
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    //开闭操作，去除噪点
    morphologyEx(dst, dst, MORPH_OPEN, element);
    morphologyEx(dst, dst, MORPH_CLOSE, element);
    return dst;
}

Mat handleFrameAndSendCmdLoop(const Mat &src, AuvManager &auv, RunningStatus &status) {
    Mat frame, tmp1;
    Mat debug = src.clone();
    GaussianBlur(src, tmp1, Size(5, 5), 3);
//    frame = Pretreatment(tmp1);
//    String result = CR(ImgDst);
//    putText(Img, result, Point(20, 30), FONT_HERSHEY_SIMPLEX, 1, Scalar(0, 255, 255), 1, 5, false);
    frame = Lpretreatment(tmp1);
    FindTubeResult tube = findTube(frame, auv, status, debug);
    if (status.isTurningRight) {
        if (tube != TURN_RIGHT && tube != NOT_FOUND) {
            status.isTurningRight = false;
            auv.goStraight();
        }
    } else {
        if (tube == TURN_RIGHT) {
            status.isTurningRight = true;
            auv.turnRight();
        } else {
            switch (tube) {
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
                case GO_STRAIGHT: {
                    //nop
                    break;
                }
            }
        }
    }
    return debug;
}

FindTubeResult findTube(const Mat &src, AuvManager &auv, RunningStatus &status, Mat &debug) {

    const int TH_MOVE = src.cols / 7;

    string str;
    vector<vector<Point>> contours1;
    vector<vector<Point>> contours2;
    vector<vector<Point>> contours3;
    vector<vector<Point>> contours;
    vector<Vec4i> hierarchay1;
    vector<Vec4i> hierarchay2;
    vector<Vec4i> hierarchay3;
    vector<Vec4i> hierarchay;
    int direction = 1;
    // double angle;
    Rect roiUp(0, 0, src.cols, src.rows / 3);
    Mat RIOImageUp(src, roiUp);
    Rect roiMedium(0, src.rows / 3, src.cols, src.rows / 3);
    Mat RIOImageM(src, roiMedium);
    Rect roiDown(0, src.rows / 3 * 2, src.cols, src.rows / 3);
    Mat RIOImageDown(src, roiDown);

    FindTubeResult result;

    findContours(RIOImageUp, contours1, hierarchay1, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
    findContours(RIOImageM, contours2, hierarchay2, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());
    findContours(RIOImageDown, contours3, hierarchay3, RETR_EXTERNAL, CHAIN_APPROX_SIMPLE, Point());

    findContours(src, contours, hierarchay, RETR_TREE, CHAIN_APPROX_SIMPLE, Point());
    int maxcontour1 = 0, maxcontour2 = 0, maxcontour3 = 0;

    //找到ROI中最大轮廓
    for (int i = 0; i < contours1.size(); i++) {
        if (maxcontour1 < contours1[i].size())
            maxcontour1 = i;
    }
    for (int i = 0; i < contours2.size(); i++) {
        if (maxcontour2 < contours2[i].size())
            maxcontour2 = i;
    }
    for (int i = 0; i < contours3.size(); i++) {
        if (maxcontour3 < contours3[i].size())
            maxcontour3 = i;
    }

    for (int i = 0; i < contours.size(); i++) {
        drawContours(debug, contours, i, Scalar(0, 255, 255), 1, 8);
    }
    drawContours(debug(roiUp), contours1, maxcontour1, Scalar(0, 255, 255), 1, 8);
    drawContours(debug(roiMedium), contours2, maxcontour2, Scalar(0, 255, 255), 1, 8);
    drawContours(debug(roiDown), contours3, maxcontour3, Scalar(0, 255, 255), 1, 8);
    //计算contour的重心
    if (!contours1.empty() && !contours2.empty() && !contours3.empty()) {
        Moments moment1, moment2, moment3;
        moment1 = moments(contours1[maxcontour1], false);
        moment2 = moments(contours2[maxcontour2], false);
        moment3 = moments(contours3[maxcontour3], false);


        Point pt1 = Point(0, 0), pt2 = Point(0, 0), pt3 = Point(0, 0);
        if (moment1.m00 != 0) {
            pt1.x = cvRound(moment1.m10 / moment1.m00);
            pt1.y = cvRound(moment1.m01 / moment1.m00);
        }
        if (moment2.m00 != 0) {
            pt2.x = cvRound(moment2.m10 / moment2.m00);
            pt2.y = cvRound(moment2.m01 / moment2.m00) + src.rows / 3;
        }
        if (moment3.m00 != 0) {
            pt3.x = cvRound(moment3.m10 / moment3.m00);
            pt3.y = cvRound(moment3.m01 / moment3.m00) + src.rows / 3 * 2;
        }


        line(debug, pt1, pt2, Scalar(0, 255, 0), 2, LINE_8);
        line(debug, pt2, pt3, Scalar(0, 255, 0), 2, LINE_8);

        float k1 = 0, k2 = 0;
        if (pt1.x != 0 && pt2.x != 0 && pt3.x != 0 && pt1.x != pt2.x && pt2.x != pt3.x) {
            k1 = (pt1.y - pt2.y) / (pt1.x - pt2.x);
            k2 = (pt2.y - pt3.y) / (pt2.x - pt3.x);
        }
        double angle1 = atan(k1) * 180 / 3.14;
        double angle2 = atan(k2) * 180 / 3.14;
        double deffrence = abs(angle1 - angle2);

        if (angle1 > -50 && angle1 < -40.0 && abs(angle2) > 70.0) {
            str += "BigRight";
//            selfInspectionSendArray[1] = 0x05;
            result = TURN_RIGHT;
        } else if (contours1[maxcontour1].size() < contours2[maxcontour2].size() / 2 &&
                   contours1[maxcontour1].size() < contours3[maxcontour3].size() / 2 &&
                   abs(contours2[maxcontour2].size() - contours3[maxcontour3].size()) <
                   contours1[maxcontour1].size() / 2) {
            if (pt1.x != 0 && pt2.x != 0 && pt3.x != 0 && pt1.x != pt2.x && pt2.x != pt3.x) {
                float k = (pt2.y - pt3.y) / (pt2.x - pt3.x);
                double angle = atan(k) * 180 / 3.14;
                if (angle > -80 && angle < -50) {
                    str += "smallright";
                    result = DRIFT_RIGHT;
                } else if (angle < 80 && angle > 50) {
                    str += "smalLeft";
                    result = DRIFT_LEFT;
                } else if (pt2.x > src.cols + TH_MOVE && pt3.x > src.cols + TH_MOVE) {
                    str += "Move_right";
                    result = TRANSLATE_RIGHT;
                } else if (pt2.x < src.cols - TH_MOVE && pt3.x < src.cols - TH_MOVE) {
                    str += "Move_left";
                    result = TRANSLATE_LEFT;
                } else if (pt2.x >= src.cols - TH_MOVE && pt2.x <= src.cols + TH_MOVE && pt3.x >= src.cols - TH_MOVE &&
                           pt3.x <= src.cols + TH_MOVE) {
                    str += "forward";
                    result = GO_STRAIGHT;
                } else {
                    str += "NOT_FOUND";
                    result = NOT_FOUND;
                }
            }
        } else if (contours2[maxcontour2].size() < contours1[maxcontour1].size() / 2 &&
                   contours2[maxcontour2].size() < contours3[maxcontour3].size() / 2 &&
                   abs(contours1[maxcontour1].size() - contours3[maxcontour3].size()) <
                   contours2[maxcontour2].size() / 2) {
            if (pt1.x != 0 && pt2.x != 0 && pt3.x != 0 && pt1.x != pt2.x && pt2.x != pt3.x) {
                float k = (pt1.y - pt3.y) / (pt1.x - pt3.x);
                double angle = atan(k) * 180 / 3.14;
                if (angle > -80 && angle < -50) {
                    str += "smallright";
                    result = DRIFT_RIGHT;
                } else if (angle < 80 && angle > 50) {
                    str += "smalLeft";
                    result = DRIFT_LEFT;
                } else if (pt1.x > src.cols + TH_MOVE && pt3.x > src.cols + TH_MOVE) {
                    str += "Move_right";
                    result = TRANSLATE_RIGHT;
                } else if (pt1.x < src.cols - TH_MOVE && pt3.x < src.cols - TH_MOVE) {
                    str += "Move_left";
                    result = TRANSLATE_LEFT;
                } else if (pt1.x >= src.cols - TH_MOVE && pt1.x <= src.cols + TH_MOVE && pt3.x >= src.cols - TH_MOVE &&
                           pt3.x <= src.cols + TH_MOVE) {
                    str += "forward";
                    result = GO_STRAIGHT;
                } else {
                    str += "NOT_FOUND";
                    result = NOT_FOUND;
                }
            }
        } else if (contours3[maxcontour3].size() < contours2[maxcontour2].size() / 2 &&
                   contours3[maxcontour3].size() < contours1[maxcontour1].size() / 2 &&
                   abs(contours2[maxcontour2].size() - contours1[maxcontour1].size()) <
                   contours3[maxcontour3].size() / 2) {
            if (pt1.x != 0 && pt2.x != 0 && pt3.x != 0 && pt1.x != pt2.x && pt2.x != pt3.x) {
                float k = (pt1.y - pt2.y) / (pt1.x - pt2.x);
                double angle = atan(k) * 180 / 3.14;
                if (angle > -80 && angle < -50) {
                    str += "smallright";
                    result = DRIFT_RIGHT;
                } else if (angle < 80 && angle > 50) {
                    str += "smalLeft";
                    result = DRIFT_LEFT;
                } else if (pt1.x > src.cols + TH_MOVE && pt2.x > src.cols + TH_MOVE) {
                    str += "Move_right";
                    result = TRANSLATE_RIGHT;
                } else if (pt1.x < src.cols - TH_MOVE && pt2.x < src.cols - TH_MOVE) {
                    str += "Move_left";
                    result = TRANSLATE_LEFT;
                } else if (pt1.x >= src.cols - TH_MOVE && pt1.x <= src.cols + TH_MOVE && pt2.x >= src.cols - TH_MOVE &&
                           pt2.x <= src.cols + TH_MOVE) {
                    str += "forward";
                    result = GO_STRAIGHT;
                } else {
                    str += "NOT_FOUND";
                    result = NOT_FOUND;
                }
            }
        } else {
            if (pt1.x > src.cols / 2 - TH_MOVE && pt1.x < src.cols / 2 + TH_MOVE && pt2.x > src.cols / 2 - TH_MOVE &&
                pt2.x < src.cols / 2 + TH_MOVE && pt3.x > src.cols / 2 - TH_MOVE && pt3.x < src.cols / 2 + TH_MOVE) {
                str += "forward";
                result = GO_STRAIGHT;
            } else if (str.empty() && pt1.x < src.cols / 2 - TH_MOVE && pt2.x < src.cols / 2 - TH_MOVE &&
                       pt3.x < src.cols / 2 - TH_MOVE) {
                str += "Move_left";
                result = TRANSLATE_LEFT;
            } else if (str.empty() && pt1.x > src.cols / 2 + TH_MOVE && pt2.x > src.cols / 2 + TH_MOVE &&
                       pt3.x > src.cols / 2 + TH_MOVE) {
                str += "Move_right";
                result = TRANSLATE_RIGHT;
            } else if (str.empty() && angle1 < 85 && angle2 < 85 && angle1 > 50 && angle2 > 50) {
                str += "smallLeft";
                result = DRIFT_LEFT;
            } else if (str.empty() && angle1 > -85 && angle2 > -85 && angle1 < -50 && angle2 < -50) {
                str += "smallright";
                result = DRIFT_RIGHT;
            }
//                else{
//                    str+="forward";
//                    selfInspectionSendArray[1]=0x07;
//                }
        }
//        if (turnOver == 1 && selfInspectionSendArray[1] != 0x07) {
//            str = "";
//            selfInspectionSendArray[1] = 0x00;
//        }
//            cout <<angle1<<"  "<<abs(angle2)<< endl;
//            cout<<str;

        //     cout << pt1.x << " " << pt1.y << "    " << pt2.x << " " << pt2.y << "   " << pt3.x << " " << pt3.y << endl;
    }


    DrawTextLeftCenterAutoColor(debug, str.c_str(), 20, 20);
//    cout << str << endl;
    return result;
}

//图片预处理
Mat Pretreatment(const Mat &src) {
    Mat dst;
    //转化为HSV
//    cvtColor(src,dst,COLOR_RGB2HSV);
//    vector<Mat> hsvSplit;
//    split(dst,hsvSplit);
//    //对HSV的亮度通道进行直方图均衡
//    equalizeHist(hsvSplit[2],hsvSplit[2]);
//    //合并三种通道
//    merge(hsvSplit,dst);
//    cvtColor(dst,dst,COLOR_HSV2BGR);
    int blockSize = 21;
    int constValue = 20;
    //cvtColor(dst,dst,COLOR_RGB2GRAY);
    //AdaptiveThereshold(dst,dst);
    //adaptiveThreshold(dst,dst,255,ADAPTIVE_THRESH_MEAN_C,THRESH_BINARY_INV,blockSize,constValue);
    inRange(dst, Scalar(0, 0, 0), Scalar(100, 75, 75), dst);
    Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
    //开闭操作，去除噪点
    morphologyEx(dst, dst, MORPH_OPEN, element);
    morphologyEx(dst, dst, MORPH_CLOSE, element);
    return dst;
}


//形状识别
String CR(const Mat &src1, Mat &debug) {
    vector<vector<Point> > contours;
    String result = "null";
    vector<Vec4i> hierarchay;
    findContours(src1, contours, hierarchay, RETR_EXTERNAL, CHAIN_APPROX_NONE);
    vector<vector<Point> > contours_poly(contours.size());
    for (int i = 0; i < contours.size(); i++) {
        approxPolyDP(Mat(contours[i]), contours_poly[i], 3, true);
        drawContours(debug, contours_poly, i, Scalar(0, 255, 255), 1, 8);
    }
    int res = 0, n = 0;
    if (!contours.empty()) {
        res = contours_poly[0].size();
    }
    cout << contours.size() << endl;
    for (int i = 1; i < contours.size(); i++) {
        if (res < contours_poly[i].size()) {
            res = contours_poly[i].size();
            n = i;
        }
    }
    //当吸附物处于图像边缘时不识别
    int Maxcol = 0, Mincol = 7, Maxrow = 0, Minrow = 7;
    for (int i = 0; i < res; i++) {
        if (Maxcol < contours_poly[n][i].x)
            Maxcol = contours_poly[n][i].x;
        if (Mincol > contours_poly[n][i].x)
            Mincol = contours_poly[n][i].x;
        if (Maxrow < contours_poly[n][i].y)
            Maxrow = contours_poly[n][i].y;
        if (Minrow > contours_poly[n][i].y)
            Minrow = contours_poly[n][i].y;
    }
//    cout<<Img.cols<<"  "<<Img.rows<<endl;
//    cout<<Maxcol<<"  "<<Mincol<<"  "<<Maxrow<<"  "<<Minrow<<endl;
    //   if (Minrow > 3 && Maxrow < Img.rows - 3 && Mincol > 3 && Maxcol < Img.cols - 3) {
    if (res > 7) {
        cout << "size=" << res << endl;
        result = "Circle";
    }
    if (res < 6 && res > 3) {
        cout << "size=" << res << endl;
        result = "Rectangle";
    }

    return result;
}

