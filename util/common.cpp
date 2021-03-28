//
// Created by kinit on 2021/3/20.
//

#include "common.h"

#ifdef __WIN32

#include "windows.h"

void msleep(int ms) {
    Sleep(ms);
}

#else

#include "future"

/* sleep for ms */
void msleep(int ms) {
    struct timeval delay = {};
    int usec = ms * 1000;
    delay.tv_sec = ms / 1000;
    delay.tv_usec = usec % 1000000;
    select(0, nullptr, nullptr, nullptr, &delay);
}

uint64_t currentTimeMillis() {
    namespace sc = std::chrono;
    auto time = sc::system_clock::now(); // get the current time
    auto since_epoch = time.time_since_epoch(); // get the duration since epoch
// I don't know what system_clock returns
// I think it's uint64_t nanoseconds since epoch
// Either way this duration_cast will do the right thing
    auto millis = sc::duration_cast<sc::milliseconds>(since_epoch);
    uint64_t now = millis.count(); // just like java (new Date()).getTime();
    return now;
}

#endif
