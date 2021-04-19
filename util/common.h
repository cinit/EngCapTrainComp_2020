//
// Created by kinit on 2021/3/20.
//

#ifndef ENGINEERTRAINUPPERCTL_COMMON_H
#define ENGINEERTRAINUPPERCTL_COMMON_H

#include "stdint.h"

#define pass ((void)0)

#if defined(__GNUC__)

#define Nullable
#define NonNull  __attribute__((nonnull))

#elif

#define Nullable
#define NonNull

#endif

/* sleep for ms */
void msleep(int ms);

uint64_t currentTimeMillis();

typedef unsigned char uchar;

#endif //ENGINEERTRAINUPPERCTL_COMMON_H
