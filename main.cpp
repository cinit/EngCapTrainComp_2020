#include "cstdio"
#include "FlowController.h"

#include "binder/LinuxSerial.h"
#include "binder/AuvManager.h"
#include "util/common.h"
#include "ipc/ClassificationManager.h"

#define SERIAL_DEV "/dev/ttyUSB0"

using namespace cv;
using namespace std;

#define DEBUG_MOTION 0

int main() {
    LinuxSerial usart(SERIAL_DEV);
    if (!usart.isOpened()) {
        printf("open serial " SERIAL_DEV " failed\n");
        return 1;
    }
    AuvManager auv(usart);
    float voltage = auv.getBatteryVoltage();
    cout << "V_bat=" << voltage << endl;
    voltage = auv.getBatteryVoltage();
    msleep(100);
    cout << "V_bat=" << voltage << endl;
    if (!DEBUG_MOTION) {
        ClassificationManager::connect();
//        VideoCapture capture("/home/kinit/Videos/AUV_00_h.mp4");
//        VideoCapture capture("/home/kinit/Videos/AUV_00_l.mp4");
//        capture.set(CAP_PROP_POS_FRAMES, 738);
//        capture.set(CAP_PROP_POS_FRAMES, 438);
        VideoCapture capture(2);
        if (!capture.isOpened()) {
            cout << "视频未打开" << endl;
            exit(1);
        } else {
            capture.set(CAP_PROP_FRAME_WIDTH, 640);
            capture.set(CAP_PROP_FRAME_HEIGHT, 480);
            capture.set(CAP_PROP_BUFFERSIZE, 1);
        }
        auv.setForwardVelocity(50);
        findTubeAndAbsorbateLoop(capture, auv, true);
    } else {
//        auv.rtlControlMotionOutput(0, 0, 0, 0);
//        msleep(20);
//        auv.stop();
//        msleep(20)
//        auv.rtlControlMotionOutput(0, 0, 0, 0);
//        msleep(20);
//        auv.stop();

//        auv.reportAdsorbate(2, AuvManager::SHAPE_RECTANGLE);

//        auv.rtlControlMotionOutput(-100,0,0,-30);
        auv.rtlControlMotionOutput(50,0, 0, 0);
        msleep(6000);
        auv.rtlControlMotionOutput(0, 0, 0, 0);
    }
}
