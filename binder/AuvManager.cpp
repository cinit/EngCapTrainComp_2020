//
// Created by kinit on 2021/3/19.
//

#include "AuvManager.h"
#include "cstdio"
#include <cstring>
#include "../util/common.h"

#define DEFLECT_VALUE 50
#define TRANSLATE_VALUE 50

#define DEBUG_PRINT_RX false

AuvManager::AuvManager(SerialInterface &usart) : usart(usart) {
}

AuvManager::AuvManager(AuvManager &that) = default;

void AuvManager::deflectLeft() {
    printf("I: AuvManager::deflect left\n");
    transactAndWaitForReply(0x01, DEFLECT_VALUE);
}

void AuvManager::deflectRight() {
    printf("I: AuvManager::deflect right\n");
    transactAndWaitForReply(0x02, DEFLECT_VALUE);
}

void AuvManager::translateLeft() {
    printf("I: AuvManager::translate left\n");
    transactAndWaitForReply(0x03, TRANSLATE_VALUE);
}

void AuvManager::translateRight() {
    printf("I: AuvManager::translate right\n");
    transactAndWaitForReply(0x04, TRANSLATE_VALUE);
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

CmdPacket &AuvManager::updateChecksum(CmdPacket &pk) {
    pk[6] = (char) (pk[1] + pk[2] + pk[3] + pk[4] + pk[5]);
    return pk;
}

int AuvManager::transactAndWaitForReply(uchar cmd, uchar arg1, uchar arg2, uchar arg3, uchar arg4) {
    CmdPacket pk = {0x75, cmd, arg1, arg2, arg3, arg4, 0xFF};
    updateChecksum(pk);
    return transactAndWaitForReply(pk);
}

int AuvManager::transactAndWaitForReply(const CmdPacket &pk) {
    int maxWait = 5;
    int maxTry = 3;
    do {
        usart.transmit(pk, 7);
        msleep(50);
        CmdPacket resp = {};
        if (nextCmdPacketAsync(resp)) {
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
                    return resp[1];
                }
            } else {
                printf("W: recv checksum mismatch");
                maxTry--;
                maxWait = 5;
            }
        }
        if ((maxTry <= 0) || (maxWait-- <= 0)) {
            printf("E: wait for cmd 0x%02x(%d,%d,%d,%d) reply timeout\n",
                   pk[1], pk[2], pk[3], pk[4], pk[5]);
            return 0xFF;
        }
    } while (true);
}

bool AuvManager::verifyChecksum(const CmdPacket &pk) {
    return (unsigned char) (pk[6]) == (unsigned char) (pk[1] + pk[2] + pk[3] + pk[4] + pk[5]);
}

void AuvManager::dropPendingRxPacket() {
    cmdbuf_len = cmdbuf_start = 0;
}

bool AuvManager::nextCmdPacketAsync(CmdPacket &pk) {
    //check buffer
    if (cmdbuf_len < 32 && (256 - cmdbuf_start - cmdbuf_len < 64)) {
        printf("Reset buffer\n");
        memcpy(cmdbuf, cmdbuf + cmdbuf_start, cmdbuf_len);
        cmdbuf_start = 0;
    }
    int rl = usart.receive(cmdbuf + cmdbuf_start, 256 - cmdbuf_start - cmdbuf_len);
    if (rl > 0) {
        if (DEBUG_PRINT_RX) {
            for (int i = 0; i < rl; i++) {
                printf("%02x, ", cmdbuf[(cmdbuf_start + i) % 256]);
            }
        }
        cmdbuf_len += rl;
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
        return true;
    }
    return false;
}




