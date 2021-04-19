//
// Created by kinit on 2021/3/19.
//

#ifndef ENGINEERTRAINUPPERCTL_AUVMANAGER_H
#define ENGINEERTRAINUPPERCTL_AUVMANAGER_H

#include "SerialInterface.h"
#include "../util/PidF.h"

typedef unsigned char uchar;
typedef unsigned char CmdPacket[7];

class AuvManager {
public:
    static const int SHAPE_RECTANGLE = 1;
    static const int SHAPE_CIRCLE = 2;

    AuvManager(AuvManager &that);

    explicit AuvManager(SerialInterface &usart);


    //上位机告知下位机开始右转弯
    void turnRight();

    //上位机告知下位机直行(右转弯结束)
    void goStraight();

    // value: [-100,100]
    void setForwardVelocity(int speed);

    //@param translationOffset pixel
    void updateCurrentError(int translationOffset, int degreeOffset, float (*out)[4]);

    bool start();

    //停止, AUX关闭所有推进器
    void stop();

    // value: [-100,100]
    void rtlControlMotionOutput(int dx, int dy, int dz, int dw);

    //上位机告知下位机检测到漏点
    //id: 吸附物计数 (从1开始)
    //shape: 吸附物形状 (1.方形 2.圆形)
    void reportAdsorbate(int id, int shape);

    float getBatteryVoltage();

protected:
    int transactAndWaitForReply(const CmdPacket &pk, CmdPacket *reply = nullptr, bool junk = true);

    int transactAndWaitForReply(uchar cmd, uchar arg1 = 0, uchar arg2 = 0, uchar arg3 = 0, uchar arg4 = 0,
                                CmdPacket *reply = nullptr, bool junk = false);

    static CmdPacket &updateChecksum(CmdPacket &pk);

    static bool verifyChecksum(const CmdPacket &pk);

    void dropPendingRxPacket();

    bool nextCmdPacketAsync(CmdPacket &pk);

private:
    SerialInterface &usart;
    unsigned char cmdbuf[256];
    int cmdbuf_start = 0, cmdbuf_len = 0;
    int maxForwardSpeed = 0;
    int currentDegreeError = 0;
    int currentTranslationError = 0;
    PidF translationY;
    PidF rotationYaw;
    int forwardSpeed = 0;
};

#endif //ENGINEERTRAINUPPERCTL_AUVMANAGER_H
