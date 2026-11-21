#ifndef _GyverPlanner_h
#define _GyverPlanner_h
#include <Arduino.h>
#include "StepperCore.h"
#define GP_MIN_US 300000
template <GS_driverType _DRV, uint8_t _AXLES>
class GPlanner {
   public:
    GPlanner() {
        setAcceleration(100);
        setMaxSpeed(300);
    }
    void addStepper(uint8_t axis, Stepper<_DRV>& stp) {
        if (axis < _AXLES) steppers[axis] = &stp;
    }
    void setBacklash(uint8_t axis, uint16_t steps) {
        blash[axis] = steps;
    }
    void enable() {
        for (uint8_t i = 0; i < _AXLES; i++) steppers[i]->enable();
    }
    void disable() {
        for (uint8_t i = 0; i < _AXLES; i++) steppers[i]->disable();
    }
    void power(bool v) {
        if (v) enable();
        else disable();
    }
    bool ready() {
        if (readyF && !status) {
            readyF = false;
            return true;
        }
        return false;
    }
    void setMaxSpeed(double speed) {
        nV = speed;
        if (!status) {
            V = nV;
            usMin = 1000000.0 / V;
            setAcceleration(na);
            changeSett = 0;
        } else changeSett = 1;
    }
    void setAcceleration(uint16_t acc) {
        na = acc;
        if (!status) {
            a = na;
            if (a != 0) us0 = 0.676 * 1000000 * sqrt(2.0 / a);
            else us0 = usMin;
            changeSett = 0;
            calcPlan();
        } else changeSett = 1;
    }
    void pause() {
        if (status == 1) status = 2;
    }
    void stop() {
        if (us == 0 || a == 0 || ((uint32_t)us << shift) > GP_MIN_US || status == 3 || !status) {
            brake();
            return;
        }
        if (status <= 2) {
            if (step > s2) {
                pause();
                return;
            }
            us <<= shift;
            stopStep = 1000000ul / us;
            stopStep = (uint32_t)stopStep * stopStep / 2 / a;
            us10 = (uint32_t)us << 10;
            us >>= shift;
            status = 4;
        }
    }
    void brake() {
        status = 0;
        us = 0;
    }
    void resume() {
        setTarget(tar);
    }
    void reset() {
        for (uint8_t i = 0; i < _AXLES; i++) steppers[i]->pos = 0;
    }
    void home() {
        int32_t pos[_AXLES] = {};
        setTarget(pos);
    }
    void setCurrent(int16_t cur[]) {
        for (uint8_t i = 0; i < _AXLES; i++) steppers[i]->pos = (int32_t)cur[i];
    }
    void setCurrent(int32_t cur[]) {
        for (uint8_t i = 0; i < _AXLES; i++) steppers[i]->pos = cur[i];
    }
    int32_t getCurrent(int axis) {
        return steppers[axis]->pos;
    }
    bool setTarget(const int32_t target[], GS_posType type = ABSOLUTE) {
        if (changeSett) {
            setMaxSpeed(nV);
            changeSett = 0;
        }
        S = 0;
        for (uint8_t i = 0; i < _AXLES; i++) {
            tar[i] = target[i];
            if (type == RELATIVE) tar[i] += steppers[i]->pos;
            dS[i] = abs(tar[i] - steppers[i]->pos);
            if (dS[i] > S) S = dS[i];
            int8_t dir = (steppers[i]->pos < tar[i]) ? 1 : -1;
            if (blash[i] && steppers[i]->dir != dir) {
                blash_buf[i] = blash[i];
            }
            steppers[i]->dir = dir;
        }
        if (S == 0) {
            readyF = true;
            brake();
            return 0;
        }
        shift = 0;
        for (; shift < 5; shift++) {
            if ((uint32_t)usMin >> shift < 200 || (0xfffffffl >> shift) < S) break;
        }
        shift--;
        int32_t subS = ((int32_t)S << shift) >> 1;
        for (int i = 0; i < _AXLES; i++) nd[i] = subS;
        if (a > 0) {
            us <<= shift;
            if (us != 0 && us < GP_MIN_US) {
                int32_t v1 = 1000000L / us;
                if ((int32_t)V * V / a - ((int32_t)v1 * v1 / a >> 1) > S) {
                    s1 = ((int32_t)S >> 1) - ((int32_t)v1 * v1 / a >> 2);
                    s2 = s1;
                } else {
                    s1 = ((int32_t)V * V - (int32_t)v1 * v1) / (2L * a);
                    s2 = S - (int32_t)V * V / (2 * a);
                }
                so1 = (int32_t)v1 * v1 / (2 * a);
            } else {
                if ((int32_t)V * V / a > S) {
                    s1 = S / 2L;
                    s2 = s1;
                } else {
                    s1 = (int32_t)V * V / (2 * a);
                    s2 = S - s1;
                }
                so1 = 0;
                us = us0;
            }
        } else {
            s1 = 0;
            s2 = S;
            us = usMin;
        }
        us10 = (uint32_t)us << 10;
        us >>= shift;
        step = substep = 0;
        readyF = false;
        status = 1;
        return 1;
    }
    bool setTarget(const int16_t target[], GS_posType type = ABSOLUTE) {
        int32_t tar[_AXLES];
        for (uint8_t i = 0; i < _AXLES; i++) tar[i] = target[i];
        return setTarget(tar, type);
    }
    int32_t getTarget(int axis) {
        return tar[axis];
    }
    void setSpeed(uint8_t axis, float speed) {
        if (speed == 0) {
            brake();
            return;
        }
        speedAxis = axis;
        steppers[axis]->dir = speed > 0 ? 1 : -1;
        us = 1000000.0 / abs(speed);
        us >>= shift;
        status = 3;
    }
    bool tick() {
        uint32_t now = micros();
        if (status > 0 && now - tmr >= us) {
            tmr = now;
            tickManual();
        }
        return status;
    }
    bool tickManual() {
        if (status == 3) {
            steppers[speedAxis]->step();
            return 1;
        }
        bool skip = 0;
        for (uint8_t i = 0; i < _AXLES; i++) {
            if (blash_buf[i]) {
                blash_buf[i]--;
                steppers[i]->step();
                steppers[i]->pos += -steppers[i]->dir;
                if (!skip) skip = 1;
            }
        }
        if (skip) return 1;
        for (uint8_t i = 0; i < _AXLES; i++) {
            nd[i] -= dS[i];
            if (nd[i] < 0) {
                nd[i] += (int32_t)S << shift;
                steppers[i]->step();
            }
        }
        if (shift)
            if (++substep & ((1 << shift) - 1)) return status;
        step++;
        if (status == 4) {
            stopStep--;
            us10 += 2ul * us10 / (4ul * stopStep + 1);
            us = (uint32_t)us10 >> 10;
            us = constrain(us, usMin, us0);
            if (stopStep <= 0 || step >= S || us >= us0) brake();
            us >>= shift;
            return status;
        }
        if (step < s1) {
#ifndef GS_FAST_PROFILE
            us10 -= 2ul * us10 / (4ul * (step + so1) + 1);
            us = (uint32_t)us10 >> 10;
            us = constrain(us, usMin, us0);
#else
            if ((step + so1) >= prfS[GS_FAST_PROFILE - 1]) us = usMin;
            else {
                int j = 0;
                while ((step + so1) >= prfS[j]) j++;
                us = prfP[j];
            }
#endif
        } else if (step < s2) us = usMin;
        else if (step < S) {
#ifndef GS_FAST_PROFILE
            us10 += 2ul * us10 / (4ul * (S - step) + 1);
            us = (uint32_t)us10 >> 10;
            us = constrain(us, usMin, us0);
#else
            if ((S - step) >= prfS[GS_FAST_PROFILE - 1]) us = usMin;
            else {
                int j = 0;
                while ((S - step) >= prfS[j]) j++;
                us = prfP[j];
            }
#endif
        } else {
            if (status == 1) readyF = true;
            brake();
        }
        us >>= shift;
        return status;
    }
    uint32_t getPeriod() {
        return us << shift;
    }
    uint8_t getStatus() {
        return status;
    }
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
#ifdef GS_FAST_PROFILE
    uint32_t prfS[GS_FAST_PROFILE], prfP[GS_FAST_PROFILE];
#endif
    uint16_t blash[_AXLES] = {}, blash_buf[_AXLES] = {};
    uint32_t us;
    int32_t tar[_AXLES], nd[_AXLES], dS[_AXLES];
    int32_t step, substep, S, s1, s2, so1;
    uint32_t tmr, us0, usMin, us10;
    uint16_t a, na;
    int16_t stopStep;
    float V, nV;
    uint8_t status = 0, speedAxis = 0;
    uint8_t shift = 0;
    bool readyF = true;
    bool changeSett = 0;
    Stepper<_DRV>* steppers[_AXLES];
};
#endif