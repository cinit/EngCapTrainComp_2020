//
// Created by kinit on 2021/3/31.
//

#ifndef ENGINEERTRAINUPPERCTL_PIDF_H
#define ENGINEERTRAINUPPERCTL_PIDF_H

#include "cstdint"

class PidF {
public:
    explicit PidF(float p, float i, float d, int max_time, float imax);

    //PidF(const PidF &that);

    PidF() = delete;

    float update(float error, float scalar = 1.0);

    void resetI();

private:
    int maxTime;
    float _kp = 0, _ki = 0, _kd = 0, _integrator = 0, _imax = 0;
    float _last_error = 0, _last_derivative = 0;
    uint64_t _last_t = 0;
};

#endif //ENGINEERTRAINUPPERCTL_PIDF_H
