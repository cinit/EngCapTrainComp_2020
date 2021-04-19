#include "cstdio"
#include "FlowController.h"

#include "binder/LinuxSerial.h"
#include "binder/AuvManager.h"
#include "util/common.h"

#define SERIAL_DEV "/dev/ttyUSB0"

using namespace cv;
using namespace std;

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
//    auv.rtlControlMotionOutput();
//    return 0;


//    VideoCapture capture("/home/kinit/Videos/AUV_00_h.mp4");
//    VideoCapture capture("/home/kinit/Videos/AUV_00_l.mp4");
//    capture.set(CAP_PROP_POS_FRAMES, 738);
//    capture.set(CAP_PROP_POS_FRAMES, 438);
    VideoCapture capture(2);
    if (!capture.isOpened()) {
        cout << "视频未打开" << endl;
        exit(1);
    } else {
        capture.set(CAP_PROP_FRAME_WIDTH, 640);
        capture.set(CAP_PROP_FRAME_HEIGHT, 480);
        capture.set(CAP_PROP_BUFFERSIZE, 1);
    }
//    auv.rtlControlMotionOutput(0, 0, 0, 0);
//    msleep(20);
//    auv.stop();
//    msleep(20)
//    auv.rtlControlMotionOutput(0, 0, 0, 0);
//    msleep(20);
//    auv.stop();
//    auv.setForwardVelocity(70);
    auv.rtlControlMotionOutput(40, 0, 0, 0);
    msleep(5000);
    auv.rtlControlMotionOutput(0, 0, 0, 0);
//    findTubeAndAbsorbateLoop(capture, auv, true);
}
