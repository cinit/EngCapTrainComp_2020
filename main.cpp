#include "cstdio"
#include "FlowController.h"

#include "binder/LinuxSerial.h"
#include "binder/AuvManager.h"
#include "util/common.h"
#include "signal.h"

#define SERIAL_DEV "/dev/ttyUSB0"

using namespace cv;
using namespace std;

volatile AuvManager *gAuvManager = nullptr;

void sigint_handler(int);

int main() {
    LinuxSerial usart(SERIAL_DEV);
    if (!usart.isOpened()) {
        printf("open serial " SERIAL_DEV " failed\n");
        return 1;
    }
//    VideoCapture capture(1);
//    if (!capture.isOpened()) {
//        printf("open video failed\n");
//        return 1;
//    }
//    VideoCapture capture("/home/kinit/Videos/AUV_00_h.mp4");
    VideoCapture capture(2);
    if (!capture.isOpened()) {
        cout << "视频未打开" << endl;
        exit(1);
    } else {
        capture.set(CAP_PROP_FRAME_WIDTH, 640);
        capture.set(CAP_PROP_FRAME_HEIGHT, 480);
        capture.set(CAP_PROP_FPS, 10);
    }
    AuvManager auv(usart);
    gAuvManager = &auv;
    signal(SIGINT, sigint_handler);
    auv.stop();
//    auv.goStraight();
    msleep(100);
    findTubeAndAbsorbateLoop(capture, auv, true);
    gAuvManager = nullptr;
}

void sigint_handler(int sig) {
    if (sig == SIGINT) {
        auto *mgr = const_cast<AuvManager *>(gAuvManager);
        if (mgr != nullptr) {
            mgr->stop();
            gAuvManager = nullptr;
        }
    }
}
