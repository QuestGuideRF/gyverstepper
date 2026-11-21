#ifndef _GyverStepper2_h
#define _GyverStepper2_h
#include <Arduino.h>
#include "StepperCore.h"
#define GS_MIN_US 300000
template <GS_driverType _DRV, GS_driverType _TYPE = STEPPER_PINS>
class GStepper2 : public Stepper<_DRV, _TYPE> {
   public:
    GStepper2(uint16_t steps, uint8_t pin1 = 255, uint8_t pin2 = 255, uint8_t pin3 = 255, uint8_t pin4 = 255, uint8_t pin5 = 255) : Stepper<_DRV, _TYPE>(pin1, pin2, pin3, pin4, pin5) {
        stepsRev = steps;
        setMaxSpeed(100);
        setAcceleration(200);
    }
    bool tick() {
        if (status) {
            uint32_t thisUs = micros();
            if (thisUs - tmr >= us) {
                tmr = thisUs;
                tickManual();
            }
        }
        return status;
    }
    bool tickManual() {
        if (!status) return 0;
        step();
#ifndef GS_NO_ACCEL
        switch (status) {
            case 1:
            case 2:
                steps++;
                if (steps < s1) {
#ifndef GS_FAST_PROFILE
                    us10 -= 2ul * us10 / (4ul * (steps + so1) + 1);
                    us = (uint32_t)us10 >> 10;
                    us = constrain(us, usMin, us0);
#else
                    if ((steps + so1) >= prfS[GS_FAST_PROFILE - 1]) us = usMin;
                    else {
                        int j = 0;
                        while ((steps + so1) >= prfS[j]) j++;
                        us = prfP[j];
                    }
#endif
                } else if (steps < s2) us = usMin;
                else if (steps < S) {
#ifndef GS_FAST_PROFILE
                    us10 += 2ul * us10 / (4ul * (S - steps) + 1);
                    us = (uint32_t)us10 >> 10;
                    us = constrain(us, usMin, us0);
#else
                    if ((S - steps) >= prfS[GS_FAST_PROFILE - 1]) us = usMin;
                    else {
                        int j = 0;
                        while ((S - steps) >= prfS[j]) j++;
                        us = prfP[j];
                    }
#endif
                } else {
                    if (revF) {
                        brake();
                        setTarget(bufT);
                        return status;
                    }
                    if (status == 1) readyF = 1;
                    brake();
                }
                return status;
            case 4:
                stopStep--;
#ifndef GS_FAST_PROFILE
                us10 += 2ul * us10 / (4ul * stopStep + 1);
                us = (uint32_t)us10 >> 10;
                us = constrain(us, usMin, us0);
#else
                if (stopStep >= prfS[GS_FAST_PROFILE - 1]) us = usMin;
                else {
                    int j = 0;
                    while (stopStep >= prfS[j]) j++;
                    us = prfP[j];
                }
#endif
                if (pos == tar || stopStep <= 0 || us >= us0) brake();
                return status;
        }
#else
        if (status <= 2 && pos == tar) {
            if (status == 1) readyF = 1;
            brake();
        }
#endif
        return status;
    }
    bool setSpeed(int32_t speed) {
        if (speed == 0) {
            brake();
            return 0;
        }
        dir = (speed > 0) ? 1 : -1;
        us = 1000000L / abs(speed);
        status = 3;
        if (autoP) enable();
        return 1;
    }
#ifdef __AVR__
    void setSpeed(int speed) {
        setSpeed((int32_t)speed);
    }
#endif
    void setSpeed(double speed) {
        if (setSpeed((int32_t)speed)) us = 1000000.0 / abs(speed);
    }
    void setSpeedDeg(int speed) {
        setSpeed((int32_t)speed * stepsRev / 360L);
    }
    void setSpeedDeg(double speed) {
        setSpeed((float)speed * stepsRev / 360L);
    }
    void setTarget(int32_t ntar, GS_posType type = ABSOLUTE) {
        if (sp0) {
            brake();
            readyF = 1;
            return;
        }
        if (changeSett) {
            usMin = usMinN;
#ifndef GS_NO_ACCEL
            V = 1000000L / usMin;
            setAcceleration(na);
#endif
            changeSett = 0;
        }
        if (type == RELATIVE) tar = ntar + pos;
        else tar = ntar;
        if (tar == pos) {
            brake();
            readyF = 1;
            return;
        }
#ifndef GS_NO_ACCEL
        revF = 0;
        S = abs(tar - pos);
        int8_t ndir = (pos < tar) ? 1 : -1;
        int32_t v1 = 0;
        if (status > 0) v1 = 1000000L / us;
        int32_t ss = 0;
        if (a > 0) ss = (int32_t)v1 * v1 / (2L * a);
        if (ss > S || (status && ndir != dir)) {
            revF = 1;
            bufT = tar;
            tar = pos + ss * dir;
            S = ss;
        }
        if (a > 0 && usMin < GS_MIN_US) {
            if ((int32_t)V * V / a - ((int32_t)v1 * v1 / a >> 1) > S) {
                if (revF) s1 = 0;
                else s1 = ((int32_t)S >> 1) - ((int32_t)v1 * v1 / a >> 2);
                s2 = s1;
            } else {
                s1 = ((int32_t)V * V - (int32_t)v1 * v1) / (2L * a);
                s2 = S - (int32_t)V * V / (2L * a);
            }
            so1 = (int32_t)v1 * v1 / (2L * a);
            if (v1 == 0) us = us0;
        } else {
            s1 = so1 = 0;
            s2 = S;
            us = usMin;
        }
        us10 = (uint32_t)us << 10;
        steps = 0;
#else
        us = usMin;
#endif
        dir = (pos < tar) ? 1 : -1;
        status = 1;
        if (autoP) enable();
        readyF = 0;
    }
    void setTargetDeg(int32_t ntar, GS_posType type = ABSOLUTE) {
        setTarget((int32_t)ntar * stepsRev / 360L, type);
    }
    void setTargetDeg(double ntar, GS_posType type = ABSOLUTE) {
        setTarget(ntar * stepsRev / 360.0, type);
    }
    int32_t getTarget() {
#ifndef GS_NO_ACCEL
        return revF ? bufT : tar;
#else
        return tar;
#endif
    }
    void setCurrent(int32_t npos) {
        pos = npos;
    }
    int32_t getCurrent() {
        return pos;
    }
    void reset() {
        pos = 0;
    }
    void setAcceleration(uint16_t acc) {
#ifndef GS_NO_ACCEL
        na = acc;
        if (!status) {
            a = na;
            if (a > 0) us0 = 0.676 * 1000000 * sqrt(2.0 / a);
            else us0 = usMin;
            changeSett = 0;
            calcPlan();
        } else changeSett = 1;
#endif
    }
    void setMaxSpeed(double speed) {
        if (speed == 0) {
            sp0 = 1;
            return;
        }
        sp0 = 0;
        usMinN = 1000000.0 / speed;
        if (!status) {
            usMin = usMinN;
#ifndef GS_NO_ACCEL
            V = (uint16_t)speed;
            setAcceleration(a);
#endif
            changeSett = 0;
        } else changeSett = 1;
    }
    void setMaxSpeedDeg(double speed) {
        setMaxSpeed(speed * stepsRev / 360.0);
    }
    void stop() {
#ifndef GS_NO_ACCEL
        if (a == 0 || us > GS_MIN_US || status == 3 || !status) {
            brake();
            return;
        }
        if (status <= 2) {
            if (steps > s2) {
                pause();
                return;
            }
            stopStep = 1000000ul / us;
            stopStep = (uint32_t)stopStep * stopStep / 2 / a;
            us10 = (uint32_t)us << 10;
            status = 4;
        }
#else
        brake();
#endif
    }
    void autoPower(bool mode) {
        autoP = mode;
    }
    void brake() {
        status = 0;
        if (autoP) disable();
    }
    void pause() {
        if (status == 1) status = 2;
    }
    void resume() {
        if (!status) setTarget(tar);
    }
    uint8_t getStatus() {
        return status;
    }
    bool ready() {
        if (!status && readyF) {
            readyF = 0;
            return 1;
        }
        return 0;
    }
    uint32_t getPeriod() {
        return us;
    }
    using Stepper<_DRV, _TYPE>::pos;
    using Stepper<_DRV, _TYPE>::dir;
    using Stepper<_DRV, _TYPE>::step;
    using Stepper<_DRV, _TYPE>::enable;
    using Stepper<_DRV, _TYPE>::disable;
   private:
    void calcPlan() {
#ifdef GS_FAST_PROFILE
        if (a > 0) {
            uint32_t sa = (uint32_t)V * V / a / 2ul;
            float dtf = sqrt(2.0 * sa / a) / GS_FAST_PROFILE;
            float s0 = a * dtf * dtf / 2.0;
            uint32_t dt = dtf * 1000000.0;
            for (int i = 0; i < GS_FAST_PROFILE; i++) {
                prfS[i] = s0 * (i + 1) * (i + 1);
                uint32_t ds = prfS[i];
                if (i > 0) ds -= prfS[i - 1];
                if (ds <= 0) prfP[i] = 0;
                else prfP[i] = (uint32_t)dt / ds;
            }
        }
#endif
    }
    uint32_t tmr = 0, us = 10000, usMin = 10000;
    int32_t tar = 0;
    uint16_t stepsRev;
    uint8_t status = 0;
    bool readyF = 0;
    bool changeSett = 0;
    bool autoP = false;
    uint32_t usMinN;
    bool sp0 = 0;
#ifndef GS_NO_ACCEL
    uint16_t a, V;
    uint16_t na;
    int16_t stopStep;
    uint32_t us0, us10;
    int32_t S, s1, s2, so1, steps;
    int32_t bufT = 0;
    bool revF = false;
#ifdef GS_FAST_PROFILE
    uint32_t prfS[GS_FAST_PROFILE], prfP[GS_FAST_PROFILE];
#endif
#endif
};
#endif