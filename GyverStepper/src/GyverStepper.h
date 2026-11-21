#ifndef _GyverStepper_h
#define _GyverStepper_h
#include <Arduino.h>
#include "GStypes.h"
#include "StepperCore.h"
#define degPerMinute(x) ((x) / 60.0f)
#define degPerHour(x) ((x) / 3600.0f)
#define _sign(x) ((x) >= 0 ? 1 : -1)
#define _MIN_SPEED_FP 5
#define _MAX_PERIOD_FP (1000000L / _MIN_SPEED_FP)
#define _MIN_STEP_SPEED (1.0 / 3600)
enum GS_runMode {
    FOLLOW_POS,
    KEEP_SPEED,
};
enum GS_smoothType {
    NO_SMOOTH,
    SMOOTH,
};
template <GS_driverType _DRV, GS_driverType _TYPE = STEPPER_PINS>
class GStepper : public Stepper<_DRV, _TYPE> {
   public:
    GStepper(int stepsPerRev, uint8_t pin1 = 255, uint8_t pin2 = 255, uint8_t pin3 = 255, uint8_t pin4 = 255, uint8_t pin5 = 255) : Stepper<_DRV, _TYPE>(pin1, pin2, pin3, pin4, pin5) {
        setMaxSpeed(300);
        setAcceleration(300);
        _stepsPerDeg = (stepsPerRev / 360.0);
    }
    bool tick() {
        if (_workState) {
            tickUs = micros();
#ifndef SMOOTH_ALGORITHM
            if (!_curMode && _accel != 0 && _maxSpeed >= _MIN_SPEED_FP) planner();
#endif
            if (_curMode && _accel != 0) smoothSpeedPlanner();
            if (stepTime && tickUs - _prevTime >= stepTime) {
                _prevTime = tickUs;
#ifdef SMOOTH_ALGORITHM
                if (!_curMode && _accel != 0 && _maxSpeed >= _MIN_SPEED_FP && !plannerSmooth()) {
                    brake();
                    return false;
                }
#endif
                if (!_curMode && _target == pos) {
                    brake();
                    return false;
                }
                step();
            }
        }
        return _workState;
    }
    void setCurrent(int32_t npos) {
        pos = npos;
        _accelSpeed = 0;
    }
    void setCurrentDeg(float npos) {
        setCurrent((float)npos * _stepsPerDeg);
    }
    int32_t getCurrent() {
        return pos;
    }
    float getCurrentDeg() {
        return ((float)pos / _stepsPerDeg);
    }
    void setTarget(int32_t npos, GS_posType type = ABSOLUTE) {
        _target = type ? (npos + pos) : npos;
        if (_target != pos) {
            if (_accel == 0 || _maxSpeed < _MIN_SPEED_FP) {
                stepTime = 1000000.0 / _maxSpeed;
                dir = (_target > pos) ? 1 : -1;
            }
            enable();
        }
    }
    void setTargetDeg(double npos, GS_posType type = ABSOLUTE) {
        setTarget((float)npos * _stepsPerDeg, type);
    }
    int32_t getTarget() {
        return _target;
    }
    float getTargetDeg() {
        return ((float)_target / _stepsPerDeg);
    }
    void setMaxSpeed(double speed) {
        speed = abs(speed);
        _maxSpeed = max(speed, _MIN_STEP_SPEED);
        if (_accel == 0 || _maxSpeed < _MIN_SPEED_FP) stepTime = 1000000.0 / _maxSpeed;
#ifdef SMOOTH_ALGORITHM
        _cmin = 1000000.0 / _maxSpeed;
        if (_n > 0) {
            _n = (float)_accelSpeed * _accelSpeed * _accelInv;
            plannerSmooth();
        }
#else
        _plannerPrd = map((int)_maxSpeed, 1000, 20000, 15000, 1000);
        _plannerPrd = constrain(_plannerPrd, 15000, 1000);
#endif
    }
    void setMaxSpeedDeg(double speed) {
        setMaxSpeed(speed * _stepsPerDeg);
    }
    void setAcceleration(uint16_t accel) {
        _accel = accel;
        if (_accel) _accelInv = 0.5f / accel;
        else _accelInv = 0;
        _accelTime = accel / 1000000.0f;
#ifdef SMOOTH_ALGORITHM
        if (_accel) _c0 = 0.676 * sqrt(2.0 / _accel) * 1000000.0;
        plannerSmooth();
#endif
    }
    void setAccelerationDeg(float accel) {
        setAcceleration(accel * _stepsPerDeg);
    }
    void autoPower(bool mode) {
        _autoPower = mode;
    }
    void stop() {
        if (_workState) {
            resetTimers();
            if (_curMode == FOLLOW_POS) {
                if (!_accel) {
                    brake();
                    return;
                }
                _accelSpeed = 1000000.0f / stepTime * dir;
                setTarget(pos + (float)_accelSpeed * _accelSpeed * _accelInv * dir);
                _stopSpeed = abs(_accelSpeed);
#ifdef SMOOTH_ALGORITHM
                _n = (float)_accelSpeed * _accelSpeed * _accelInv;
#endif
            } else {
                setSpeed(0);
            }
        }
    }
    void reset() {
        brake();
        setCurrent(0);
    }
    void setSpeed(float speed, bool smooth = false) {
        _speed = speed;
        _stopF = (_speed == 0);
        if (_speed == 0 && _accelSpeed == 0) return;
        dir = (_speed > 0) ? 1 : -1;
        if (abs(_speed) < _MIN_STEP_SPEED) _speed = _MIN_STEP_SPEED * dir;
        if (_accel != 0) {
            if (_accelSpeed != _speed) {
                int speed1 = (int)abs(_speed);
                int speed2 = (int)abs(_accelSpeed);
                _speedPlannerPrd = map(max(speed1, speed2), 1000, 20000, 15000, 2000);
                _speedPlannerPrd = constrain(_speedPlannerPrd, 15000, 2000);
                stepTime = abs(1000000.0 / _accelSpeed);
            }
        } else {
            if (speed == 0) {
                brake();
                return;
            }
            _accelSpeed = _speed;
            stepTime = abs(1000000.0 / _speed);
        }
        enable();
    }
    void setSpeedDeg(float speed, bool smooth = false) {
        setSpeed(_stepsPerDeg * speed);
    }
    float getSpeed() {
        return (1000000.0 / stepTime * dir);
    }
    float getSpeedDeg() {
        return ((float)getSpeed() / _stepsPerDeg);
    }
    void setRunMode(GS_runMode mode) {
        if (_curMode != mode) resetTimers();
        _curMode = mode;
    }
    bool getState() {
        return _workState;
    }
    void enable() {
        _workState = true;
        _stopSpeed = 0;
        resetTimers();
        Stepper<_DRV, _TYPE>::enable();
    }
    void brake() {
        _workState = false;
        _stopSpeed = 0;
        resetMotor();
        if (_autoPower) Stepper<_DRV, _TYPE>::disable();
    }
    uint32_t getMinPeriod() {
        float curSpeed;
        if (_curMode == KEEP_SPEED) {
            curSpeed = abs(_speed);
            if (abs(_accelSpeed) > curSpeed) curSpeed = abs(_accelSpeed);
        } else curSpeed = _maxSpeed;
        return (1000000.0 / curSpeed);
    }
    uint32_t stepTime = 10000;
    using Stepper<_DRV, _TYPE>::pos;
    using Stepper<_DRV, _TYPE>::dir;
    using Stepper<_DRV, _TYPE>::step;
    using Stepper<_DRV, _TYPE>::enable;
    using Stepper<_DRV, _TYPE>::disable;
   private:
    void resetMotor() {
        _accelSpeed = 0;
#ifdef SMOOTH_ALGORITHM
        _n = 0;
#endif
    }
    void resetTimers() {
        _speedPlannerTime = _plannerTime = _prevTime = micros();
    }
#ifdef SMOOTH_ALGORITHM
    bool plannerSmooth() {
        int32_t err = _target - pos;
        int32_t stepsToStop = (float)_accelSpeed * _accelSpeed * _accelInv;
        if (err == 0 && stepsToStop <= 1) return false;
        if (err > 0) {
            if (_n > 0) {
                if ((stepsToStop >= err) || dir == -1)
                    _n = -stepsToStop;
            } else if (_n < 0) {
                if ((stepsToStop < err) && dir == 1)
                    _n = -_n;
            }
        } else if (err < 0) {
            if (_n > 0) {
                if ((stepsToStop >= -err) || dir == 1)
                    _n = -stepsToStop;
            } else if (_n < 0) {
                if ((stepsToStop < -err) && dir == -1)
                    _n = -_n;
            }
        }
        if (_n == 0) {
            _cn = _c0;
            dir = _sign(err);
        } else {
            _cn = _cn - ((2.0 * _cn) / ((4.0 * _n) + 1));
            _cn = max(_cn, _cmin);
        }
        _n++;
        stepTime = _cn;
        _accelSpeed = 1000000.0 / _cn;
        if (dir == -1) _accelSpeed = -_accelSpeed;
        return true;
    }
    int32_t _n = 0;
    float _c0 = 0.0;
    float _cn = 0.0;
    float _cmin = 1.0;
#else
    void planner() {
        if (tickUs - _plannerTime >= _plannerPrd) {
            _plannerTime += _plannerPrd;
            int32_t err = _target - pos;
            bool thisDir = (_accelSpeed * _accelSpeed * _accelInv >= abs(err));
            _accelSpeed += (_accelTime * _plannerPrd * (thisDir ? -_sign(_accelSpeed) : _sign(err)));
            if (_stopSpeed == 0) _accelSpeed = constrain(_accelSpeed, -_maxSpeed, _maxSpeed);
            else _accelSpeed = constrain(_accelSpeed, -_stopSpeed, _stopSpeed);
            if (abs(_accelSpeed) > _MIN_SPEED_FP) stepTime = abs(1000000.0 / _accelSpeed);
            else stepTime = _MAX_PERIOD_FP;
            dir = _sign(_accelSpeed);
        }
    }
    uint16_t _plannerPrd = 15000;
#endif
    float _accelTime = 0;
    uint16_t _speedPlannerPrd = 15000;
    uint32_t _speedPlannerTime = 0;
    uint32_t _plannerTime = 0;
    void smoothSpeedPlanner() {
        if (tickUs - _speedPlannerTime >= _speedPlannerPrd) {
            _speedPlannerTime = tickUs;
            _accelSpeed += (_accelTime * _speedPlannerPrd * _sign(_speed - _accelSpeed));
            dir = _sign(_accelSpeed);
            stepTime = abs(1000000.0 / _accelSpeed);
            if (_stopF && abs(_accelSpeed) <= _MIN_STEP_SPEED) brake();
        }
    }
    bool _stopF = 0;
    float _stepsPerDeg;
    uint32_t _prevTime = 0;
    float _accelSpeed = 0;
    int32_t _target = 0;
    volatile uint32_t tickUs = 0;
    bool _workState = false;
    bool _autoPower = false;
    float _stopSpeed = 0;
    float _maxSpeed = 300;
    float _speed = 0;
    uint16_t _accel = 0;
    float _accelInv = 0;
    GS_runMode _curMode = FOLLOW_POS;
};
#endif