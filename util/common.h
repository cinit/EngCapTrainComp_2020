//
// Created by kinit on 2021/3/20.
//

#ifndef ENGINEERTRAINUPPERCTL_COMMON_H
#define ENGINEERTRAINUPPERCTL_COMMON_H

#include "stdint.h"

#if defined(__GNUC__)

#define Nullable
#define NonNull  __attribute__((nonnull))

#elif

#define Nullable
#define NonNull

#endif

/* sleep for ms */
void msleep(int ms);

typedef unsigned char uchar;

#endif //ENGINEERTRAINUPPERCTL_COMMON_H
