//
// Created by kinit on 2021/3/19.
//

#include "AuvManager.h"
#include "cstdio"
#include <cstring>
#include <cstdlib>
#include "../util/common.h"

#define DEBUG_PRINT_RX 0

const float PIXEL_COEFF = 1.0;

const int TIME_UNIT = 100;
const int MAX_TIME_INTERVAL = 1000;

inline int in_range(int min, int value, int max) {
    if (value < min) {
        value = min;
    }
    if (value > max) {
        value = max;
    }
    return value;
}

AuvManager::AuvManager(SerialInterface &usart) :
        usart(usart),
        translationY(0.3, 0, 0, 1000, 10),
        rotationYaw(0.6, 0, 0.001, 1000, 10) {
}

AuvManager::AuvManager(AuvManager &that) = default;

bool AuvManager::start() {
    float test = getBatteryVoltage();
    return (test == test);
}

void AuvManager::setForwardVelocity(int speed) {
    forwardSpeed = speed;

}

void AuvManager::updateCurrentError(int translationOffset, int degreeOffset, float (*outptr)[4]) {
    if (forwardSpeed != 0) {
        float outX = (float) forwardSpeed;
        float outY = translationY.update((float) translationOffset);
        float outZ = 0;
        float outW = rotationYaw.update((float) degreeOffset);
        int d = (in_range(0, std::abs(outY), 100) + in_range(0, std::abs(outW), 100)) / 2;
        outX -= d;
        if (outptr != nullptr) {
            float *out = *outptr;
            out[0] = outX;
            out[1] = outY;
            out[2] = outZ;
            out[3] = outW;
        }
        rtlControlMotionOutput((int) outX, (int) outY, (int) outZ, (int) outW);
    }
}

// value: [-100,100]
void AuvManager::rtlControlMotionOutput(int dx, int dy, int dz, int dw) {
    dx = in_range(-100, dx, 100);
    dy = in_range(-100, dy, 100);
    dz = in_range(-100, dz, 100);
    dw = in_range(-100, dw, 100);
    transactAndWaitForReply(0x30, (unsigned char) (dx + 100), (unsigned char) (dy + 100),
                            (unsigned char) (dz + 100), (unsigned char) (dw + 100));
}

void AuvManager::turnRight() {
    printf("I: AuvManager::turnRight>>>\n");
    transactAndWaitForReply(0x05);
}

void AuvManager::goStraight() {
    printf("I: AuvManager::goStraight\n");
    transactAndWaitForReply(0x07, 0);
}

void AuvManager::stop() {
    printf("I: AuvManager::stop\n");
    transactAndWaitForReply(0x08);
}

void AuvManager::reportAdsorbate(int id, int shape) {
    printf("I: AuvManager::reportAdsorbate(%d, %d)\n", id, shape);
    transactAndWaitForReply(0x10, (unsigned char) id, (unsigned char) shape);
}

float AuvManager::getBatteryVoltage() {
    CmdPacket pk;
    if (transactAndWaitForReply(0x21, 0, 0, 0, 0, &pk, true) == 0) {
        unsigned char h = pk[4];
        unsigned char l = pk[5];
        int val = (h << 8 | l) & 0xFFFF;
        return float(val) / 1000.0f;
    } else {
        float ret;
        *((unsigned int *) &ret) = 0x7FFFFFFF;
        return ret;
    }
}

CmdPacket &AuvManager::updateChecksum(CmdPacket &pk) {
    pk[6] = (char) (pk[1] + pk[2] + pk[3] + pk[4] + pk[5]);
    return pk;
}

int AuvManager::transactAndWaitForReply(uchar cmd, uchar arg1, uchar arg2, uchar arg3, uchar arg4,
                                        CmdPacket *reply, bool junk) {
    CmdPacket pk = {0x75, cmd, arg1, arg2, arg3, arg4, 0xFF};
    updateChecksum(pk);
    return transactAndWaitForReply(pk, reply, junk);
}

int AuvManager::transactAndWaitForReply(const CmdPacket &pk, CmdPacket *reply, bool junk) {
//    return 1;
    int maxWait = 10;
    int maxTry = 2;
    dropPendingRxPacket();
    do {
        usart.transmit(pk, 7);
        CmdPacket resp = {};
        while (!nextCmdPacketAsync(resp) && maxWait > 0) {
            msleep(5);
            maxWait--;
        }
        if (maxWait <= 0) {
            maxTry--;
//            if (DEBUG_PRINT_RX) {
//                printf("stage 1 out\n");
//            }
            continue;
        }
        if (junk) {
            maxWait = 10;
            do {
                maxWait--;
                msleep(5);
            } while ((!nextCmdPacketAsync(resp)) && maxWait > 0);
        }
        if (maxWait <= 0) {
//            if (DEBUG_PRINT_RX) {
//                printf("stage 2 out\n");
//            }
            maxTry--;
            continue;
        }
        if (verifyChecksum(resp)) {
            if (resp[1] != 0) {
                printf("E: cmd 0x%02x(%d,%d,%d,%d) get invalid resp: 0x%02x(%d,%d,%d,%d)\n",
                       pk[1], pk[2], pk[3], pk[4], pk[5], resp[1], resp[2], resp[3], resp[4], resp[5]);
                maxTry--;
                maxWait = 5;
            } else {
                if (resp[2] != 0) {
                    printf("I: cmd 0x%02x(%d,%d,%d,%d) result 0x%02x\n",
                           pk[1], pk[2], pk[3], pk[4], pk[5], resp[2]);
                }
                if (reply != nullptr) {
                    memcpy(reply, resp, 7);
                }
                return resp[1];
            }
        } else {
            printf("W: recv checksum mismatch\n");
            maxTry--;
            maxWait = 5;
        }
        if ((maxTry <= 0) || (maxWait-- <= 0)) {
            printf("E: wait for cmd 0x%02x(%d,%d,%d,%d) reply timeout\n",
                   pk[1], pk[2], pk[3], pk[4], pk[5]);
            return 0xFF;
        }
    } while (maxTry > 0);
    printf("E: wait for cmd 0x%02x(%d,%d,%d,%d) reply timeout\n",
           pk[1], pk[2], pk[3], pk[4], pk[5]);
    return 0xFF;
}

bool AuvManager::verifyChecksum(const CmdPacket &pk) {
    return (unsigned char) (pk[6]) == (unsigned char) (pk[1] + pk[2] + pk[3] + pk[4] + pk[5]);
}

void AuvManager::dropPendingRxPacket() {
    char dummy[64];
    usart.receive(dummy, 64);
    cmdbuf_len = cmdbuf_start = 0;
    if (DEBUG_PRINT_RX) {
        printf("drop rx\n");
    }
}

bool AuvManager::nextCmdPacketAsync(CmdPacket &pk) {
    //check buffer
    if (cmdbuf_len < 32 && (256 - cmdbuf_start - cmdbuf_len < 64)) {
        printf("Reset buffer\n");
        memcpy(cmdbuf, cmdbuf + cmdbuf_start, cmdbuf_len);
        cmdbuf_start = 0;
    }
    int rl = usart.receive(cmdbuf + cmdbuf_start + cmdbuf_len, 256 - cmdbuf_start - cmdbuf_len);
    cmdbuf_len += rl;
    if (rl > 0) {
        if (DEBUG_PRINT_RX) {
            for (int i = 0; i < rl; i++) {
                printf("%02x, ", cmdbuf[(cmdbuf_start + i) % 256]);
            }
        }
        if (DEBUG_PRINT_RX) {
            printf("\n");
        }
    }
    //skip bad value
    while (cmdbuf_len > 0 && *(cmdbuf + cmdbuf_start) != 0x75) {
        cmdbuf_len--;
        cmdbuf_start++;
    }
    if (cmdbuf_len >= 7 && *(cmdbuf + cmdbuf_start) == 0x75) {
        //start from 6
        memcpy(pk, cmdbuf + cmdbuf_start, 7);
        cmdbuf_len -= 7;
        cmdbuf_start += 7;
//        if (DEBUG_PRINT_RX) {
//            printf("consume7\n");
//        }
        return true;
    }
    return false;
}




