#ifndef _GyverPlanner2_h
#define _GyverPlanner2_h
#include <Arduino.h>
#include "FIFO.h"
#include "StepperCore.h"
#define GP_MIN_US 300000
template <GS_driverType _DRV, uint8_t _AXLES, uint8_t _BUF = 32>
class GPlanner2 {
   public:
    GPlanner2() {
        setAcceleration(100);
        setMaxSpeed(300);
    }
    void addStepper(uint8_t axis, Stepper<_DRV>& stp) {
        if (axis < _AXLES) steppers[axis] = &stp;
    }
    void setBacklash(uint8_t axis, uint16_t steps) {
        blash[axis] = steps;
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
    void setDtA(float newDta) {
        dtA = newDta;
    }
    void stop() {
        if (((uint32_t)us << shift) == 0 || status == 0 || status == 4) return;
        if (a == 0 || ((uint32_t)us << shift) > GP_MIN_US) {
            brake();
            return;
        }
        us <<= shift;
        stopStep = 1000000ul / us;
        stopStep = (uint32_t)stopStep * stopStep / 2 / a;
        us10 = (uint32_t)us << 10;
        us >>= shift;
        status = 4;
    }
    void start() {
        if (status == 0) status = 1;
    }
    void brake() {
        status = 0;
        us = 0;
        for (int i = 0; i < _AXLES; i++) bufP[i].set(0, steppers[i]->pos);
        bufV.set(0, 0);
    }
    void resume() {
        if (status == 0) status = 1;
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
    void reset() {
        for (uint8_t i = 0; i < _AXLES; i++) steppers[i]->pos = 0;
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
    int32_t getTarget(int axis) {
        return bufP[axis].get(1);
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
        status = 5;
    }
    bool tick() {
        checkBuffer();
        uint32_t now = micros();
        if (status > 1 && now - tmr >= us) {
            tmr = now;
            tickManual();
        }
        return status > 1;
    }
    bool tickManual() {
        if (status == 5) {
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
            if (++substep & ((1 << shift) - 1)) return status > 1;
        step++;
        if (status == 4) {
            stopStep--;
            us10 += 2ul * us10 / (4ul * stopStep + 1);
            us = (uint32_t)us10 >> 10;
            us = constrain(us, usMin, us0);
            if (step >= S) {
                next();
                setTarget();
                us = us10 >> 10;
            }
            if (stopStep <= 0 || us >= us0) brake();
            us >>= shift;
            return status > 1;
        }
        if (a > 0) {
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
                us10 += 2ul * us10 / (4ul * (S - step + so2) + 1);
                us = (uint32_t)us10 >> 10;
                us = constrain(us, usMin, us0);
#else
                if ((S - step + so2) >= prfS[GS_FAST_PROFILE - 1]) us = usMin;
                else {
                    int j = 0;
                    while ((S - step + so2) >= prfS[j]) j++;
                    us = prfP[j];
                }
#endif
            }
        }
        if (step >= S) {
            if (status == 3) {
                readyF = true;
                status = 0;
            } else status = 1;
            next();
        }
        us >>= shift;
        return (status > 1);
    }
    bool checkBuffer() {
        if (status == 1) {
            if (bufL.available() > 1) {
                if (--block < 0) block = 0;
                if (a > 0 && (bufV.get(0) == 0 || (block == 0 && !changeSett && us > blockCalc))) calculateBlock();
                if (setTarget()) return 1;
            }
        }
        return 0;
    }
    bool ready() {
        if (readyF && !status) {
            readyF = false;
            return 1;
        }
        return 0;
    }
    bool available() {
        return bufV.availableForWrite();
    }
    void addTarget(int32_t tar[], uint8_t l, GS_posType type = ABSOLUTE) {
        if (type == ABSOLUTE)
            for (int i = 0; i < _AXLES; i++) bufP[i].add(tar[i]);
        else
            for (int i = 0; i < _AXLES; i++) bufP[i].add(tar[i] + bufP[i].getLast());
        bufL.add(l);
        bufV.add(0);
        bufS.add(0);
    }
    void addTarget(int16_t tar[], uint8_t l, GS_posType type = ABSOLUTE) {
        int32_t ntar[_AXLES];
        for (int i = 0; i < _AXLES; i++) ntar[i] = (int32_t)tar[i];
        addTarget(ntar, l, type);
    }
    uint32_t getPeriod() {
        return us << shift;
    }
    uint8_t getStatus() {
        return status;
    }
    void clearBuffer() {
        for (int i = 0; i < _AXLES; i++) bufP[i].clear();
        bufL.clear();
        bufV.clear();
        bufS.clear();
    }
   private:
    void calculateBlock() {
        if (changeSett && bufV.get(0) == 0) {
            noInterrupts();
            uint8_t stBuf = status;
            status = 0;
            setMaxSpeed(nV);
            status = stBuf;
            interrupts();
            changeSett = 0;
        }
        uint32_t calcTime = micros();
        block = _BUF / 2;
        uint32_t nextS = calcS(0);
        for (uint16_t i = 0; i < bufV.available() - 1; i++) {
            int32_t dn0[_AXLES];
            for (int j = 0; j < _AXLES; j++) dn0[j] = bufP[j].get(i + 1) - bufP[j].get(i);
            uint32_t S1 = nextS;
            bufS.set(i, S1);
            if (bufL.get(i + 1) == 1) break;
            if (S1 == 0) continue;
            if (i < bufV.available() - 2) {
                int32_t multSum = 0;
                for (int j = 0; j < _AXLES; j++) multSum += (int32_t)dn0[j] * (bufP[j].get(i + 2) - bufP[j].get(i + 1));
                nextS = calcS(i + 1);
                if (nextS == 0) continue;
                float cosa = -(float)multSum / S1 / nextS;
                float Vm;
                if (cosa < -0.95) Vm = V;
                else {
                    Vm = (float)a * dtA / sqrt(2.0 * (1 + cosa));
                    Vm = min(Vm, V);
                }
                bufV.set(i + 1, Vm);
            }
        }
        for (uint16_t i = 0; i < bufV.available() - 1; i++) {
            uint32_t v0 = bufV.get(i);
            uint32_t v1 = bufV.get(i + 1);
            uint32_t maxV = sqrt(2L * a * bufS.get(i) + (uint32_t)v0 * v0);
            if (v1 > v0 && maxV < v1) bufV.set(i + 1, maxV);
            else if (v1 < v0 && maxV > v1) {
                int16_t count = 0;
                while (true) {
                    uint32_t minV = bufV.get(i + count + 1);
                    minV *= (uint32_t)minV;
                    minV = sqrt(2ul * a * bufS.get(i + count) + minV);
                    if (minV >= bufV.get(i + count)) break;
                    else bufV.set(i + count, minV);
                    count--;
                }
            }
        }
        blockCalc = (blockCalc + micros() - calcTime) / 2;
    }
    void next() {
        for (int i = 0; i < _AXLES; i++) bufP[i].next();
        bufL.next();
        bufV.next(FIFO_WIPE);
        bufS.next();
    }
    bool setTarget() {
        tmr = micros();
        for (uint8_t i = 0; i < _AXLES; i++) {
            dS[i] = abs(bufP[i].get(1) - steppers[i]->pos);
            int8_t dir = steppers[i]->pos < bufP[i].get(1) ? 1 : -1;
            if (blash[i] && steppers[i]->dir != dir) {
                blash_buf[i] = blash[i];
            }
            steppers[i]->dir = dir;
        }
        if (a == 0) S = calcS(0);
        else S = bufS.get(0);
        if (S == 0) {
            status = 1;
            next();
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
            int32_t v1 = bufV.get(0);
            int32_t v2 = bufV.get(1);
            if ((V * V - ((int32_t)v1 * v1 >> 1) - ((int32_t)v2 * v2 >> 1)) / a > S) {
                s1 = ((int32_t)S >> 1) + (((int32_t)v2 * v2 >> 2) - ((int32_t)v1 * v1 >> 2)) / a;
                s2 = 0;
            } else {
                s1 = ((int32_t)V * V - (int32_t)v1 * v1) / (2L * a);
                s2 = S - ((int32_t)V * V - (int32_t)v2 * v2) / (2L * a);
            }
            so1 = (int32_t)v1 * v1 / (2L * a);
            so2 = (int32_t)v2 * v2 / (2L * a);
            if (status != 4) {
                if (v1 == 0) us = us0;
                else us = 1000000ul / v1;
            }
        } else {
            s1 = 0;
            s2 = S;
            so1 = so2 = 0;
            us = usMin;
        }
        step = substep = 0;
        readyF = false;
        if (status != 4) {
            if (bufL.get(1) == 1) status = 3;
            else status = 2;
            us10 = us << 10;
            us >>= shift;
        }
        return 1;
    }
    uint32_t calcS(int i) {
        uint32_t sqSum = 0;
        int32_t dj;
        for (int j = 0; j < _AXLES; j++) {
            dj = bufP[j].get(i + 1) - bufP[j].get(i);
            sqSum += (int32_t)dj * dj;
        }
        return sqrt(sqSum);
    }
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
    int32_t nd[_AXLES], dS[_AXLES];
    int32_t step, substep, S, s1, s2, so1, so2;
    uint32_t tmr, us0, usMin, us10;
    uint16_t a, na;
    int16_t stopStep;
    float V, nV;
    uint8_t status = 0, speedAxis = 0, maxAx;
    uint8_t shift = 0;
    bool readyF = true;
    float dtA = 0.3;
    int8_t block = 100;
    uint32_t blockCalc = 20000;
    bool changeSett = 0;
    Stepper<_DRV>* steppers[_AXLES];
    FIFO<int32_t, _BUF> bufP[_AXLES];
    FIFO<uint8_t, _BUF> bufL;
    FIFO<uint16_t, _BUF> bufV;
    FIFO<uint32_t, _BUF> bufS;
};
#endif