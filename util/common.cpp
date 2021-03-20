//
// Created by kinit on 2021/3/20.
//

#include "common.h"

#include "future"

/* sleep for ms */
void msleep(int ms) {
    struct timeval delay = {};
    int usec = ms * 1000;
    delay.tv_sec = ms / 1000;
    delay.tv_usec = usec % 1000000;
    select(0, nullptr, nullptr, nullptr, &delay);
}
