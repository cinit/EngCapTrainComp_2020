//
// Created by kinit on 2021/3/31.
//

#include "PidF.h"
#include <cmath>
#include "common.h"

const float RC = 1 / (2 * M_PI * 20);

PidF::PidF(float p, float i, float d, int max_time, float imax) :
        _kp(p),
        _ki(i),
        _kd(d),
        _imax(std::abs(imax)),
        maxTime(max_time),
        _last_derivative(0) {
}

void PidF::resetI() {
    _integrator = 0;
    _last_derivative = 0;
}

float PidF::update(float error, float scalar) {
    uint64_t now = currentTimeMillis();
    uint64_t dt = now - _last_t;
    float output = 0;
    if (_last_t == 0 || dt > maxTime) {
        dt = 0;
        resetI();
    }
    _last_t = now;
    output += error * _kp;
    float derivative;
    if (std::abs(_kd) > 0 && dt > 0) {
        if (_last_derivative != 0) {
            derivative = 0;
            _last_derivative = 0;
        } else {
            derivative = (error - _last_error) / dt;
        }
        derivative = _last_derivative + ((dt / (RC + dt)) * (derivative - _last_derivative));
        _last_error = error;
        _last_derivative = derivative;
        output += _kd * derivative;
    }
    output *= scalar;
    if (std::abs(_ki) > 0 && dt > 0) {
        _integrator += (error * _ki) * scalar * dt;
        if (_integrator < -_imax) {
            _integrator = -_imax;
        } else if (_integrator > _imax) {
            _integrator = _imax;
        }
        output += _integrator;
    }
    return output;
}
