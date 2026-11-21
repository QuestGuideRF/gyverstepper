#ifndef _StepperCore_h
#define _StepperCore_h
#include <Arduino.h>
#include "GStypes.h"
#ifndef DRIVER_STEP_TIME
#define DRIVER_STEP_TIME 4
#endif
#define _PINS_AMOUNT ((_TYPE == STEPPER_PINS) ? (_DRV == 0 ? 2 : 4) : (0))
template <GS_driverType _DRV, GS_driverType _TYPE = STEPPER_PINS>
class Stepper {
   public:
    Stepper(uint8_t pin1 = 255, uint8_t pin2 = 255, uint8_t pin3 = 255, uint8_t pin4 = 255, uint8_t pin5 = 255) {
        if (_TYPE == STEPPER_PINS) {
            if (_DRV == STEPPER2WIRE) {
                configurePin(0, pin1);
                configurePin(1, pin2);
                if (pin3 != 255) {
                    _enPin = pin3;
                    pinMode(_enPin, OUTPUT);
                }
            } else {
                configurePin(0, pin1);
                configurePin(1, pin2);
                configurePin(2, pin3);
                configurePin(3, pin4);
                if (pin5 != 255) {
                    _enPin = pin5;
                    pinMode(_enPin, OUTPUT);
                }
            }
        }
    }
    void step() {
        pos += dir;
        if (_DRV == STEPPER2WIRE) {
            stepDir();
        } else {
            thisStep += (_globDir ? dir : -dir);
            step4();
        }
    }
    void invertEn(bool val) {
        _enDir = val;
    }
    void reverse(bool val) {
        if (_DRV == STEPPER2WIRE) {
            if (_TYPE == STEPPER_PINS && _globDir != val) setPin(1, (dir > 0) ^ val);
        }
        _globDir = val;
    }
    void disable() {
        if (_TYPE == STEPPER_PINS) {
            if (_DRV == STEPPER4WIRE || _DRV == STEPPER4WIRE_HALF) {
                setPin(0, 0);
                setPin(1, 0);
                setPin(2, 0);
                setPin(3, 0);
            }
            if (_enPin != 255) digitalWrite(_enPin, !_enDir);
        } else {
            if (*_power) _power(0);
            if (*_step && (_DRV == STEPPER4WIRE || _DRV == STEPPER4WIRE_HALF)) _step(0);
        }
    }
    void enable() {
        if (_TYPE == STEPPER_PINS) {
            if (_DRV == STEPPER4WIRE || _DRV == STEPPER4WIRE_HALF) step4();
            if (_enPin != 255) digitalWrite(_enPin, _enDir);
        } else {
            if (*_power) _power(1);
            if (*_step && (_DRV == STEPPER4WIRE || _DRV == STEPPER4WIRE_HALF)) step4();
        }
    }
    void power(bool state) {
        if (state) enable();
        else disable();
    }
    void attachStep(void (*handler)(uint8_t)) {
        _step = handler;
    }
    void attachPower(void (*handler)(bool)) {
        _power = handler;
    }
    int32_t pos = 0;
    int8_t dir = 1;
   private:
    void configurePin(int num, uint8_t pin) {
        pinMode(pin, OUTPUT);
#ifdef __AVR__
        _port_reg[num] = portOutputRegister(digitalPinToPort(pin));
        _bit_mask[num] = digitalPinToBitMask(pin);
#else
        _pins[num] = pin;
#endif
    }
    void setPin(int num, bool state) {
#ifdef __AVR__
        if (state) *_port_reg[num] |= _bit_mask[num];
        else *_port_reg[num] &= ~_bit_mask[num];
#elif defined(ESP8266)
        if (state) GPOS = (1 << _pins[num]);
        else GPOC = (1 << _pins[num]);
#else
        digitalWrite(_pins[num], state);
#endif
    }
    void step4() {
        if (_TYPE == STEPPER_PINS) {
            if (_DRV == STEPPER4WIRE) {
                switch (thisStep & 0b11) {
                    case 0:
                        setPin(0, 1);
                        setPin(1, 0);
                        setPin(2, 1);
                        setPin(3, 0);
                        break;
                    case 1:
                        setPin(0, 0);
                        setPin(1, 1);
                        setPin(2, 1);
                        setPin(3, 0);
                        break;
                    case 2:
                        setPin(0, 0);
                        setPin(1, 1);
                        setPin(2, 0);
                        setPin(3, 1);
                        break;
                    case 3:
                        setPin(0, 1);
                        setPin(1, 0);
                        setPin(2, 0);
                        setPin(3, 1);
                        break;
                }
            } else if (_DRV == STEPPER4WIRE_HALF) {
                switch (thisStep & 0b111) {
                    case 0:
                        setPin(0, 1);
                        setPin(1, 0);
                        setPin(2, 0);
                        setPin(3, 0);
                        break;
                    case 1:
                        setPin(0, 1);
                        setPin(1, 0);
                        setPin(2, 1);
                        setPin(3, 0);
                        break;
                    case 2:
                        setPin(0, 0);
                        setPin(1, 0);
                        setPin(2, 1);
                        setPin(3, 0);
                        break;
                    case 3:
                        setPin(0, 0);
                        setPin(1, 1);
                        setPin(2, 1);
                        setPin(3, 0);
                        break;
                    case 4:
                        setPin(0, 0);
                        setPin(1, 1);
                        setPin(2, 0);
                        setPin(3, 0);
                        break;
                    case 5:
                        setPin(0, 0);
                        setPin(1, 1);
                        setPin(2, 0);
                        setPin(3, 1);
                        break;
                    case 6:
                        setPin(0, 0);
                        setPin(1, 0);
                        setPin(2, 0);
                        setPin(3, 1);
                        break;
                    case 7:
                        setPin(0, 1);
                        setPin(1, 0);
                        setPin(2, 0);
                        setPin(3, 1);
                        break;
                }
            }
        } else if (*_step) {
            if (_DRV == STEPPER4WIRE) {
                switch (thisStep & 0b11) {
                    case 0:
                        _step(0b1010);
                        break;
                    case 1:
                        _step(0b0110);
                        break;
                    case 2:
                        _step(0b0101);
                        break;
                    case 3:
                        _step(0b1001);
                        break;
                }
            } else if (_DRV == STEPPER4WIRE_HALF) {
                switch (thisStep & 0b111) {
                    case 0:
                        _step(0b1000);
                        break;
                    case 1:
                        _step(0b1010);
                        break;
                    case 2:
                        _step(0b0010);
                        break;
                    case 3:
                        _step(0b0110);
                        break;
                    case 4:
                        _step(0b0100);
                        break;
                    case 5:
                        _step(0b0101);
                        break;
                    case 6:
                        _step(0b0001);
                        break;
                    case 7:
                        _step(0b1001);
                        break;
                }
            }
        }
    }
    void stepDir() {
        if (_TYPE == STEPPER_PINS) {
            if (_pdir != dir) {
                _pdir = dir;
                setPin(1, (dir > 0) ^ _globDir);
                if (DRIVER_STEP_TIME > 0) delayMicroseconds(DRIVER_STEP_TIME);
            }
            setPin(0, 1);
            if (DRIVER_STEP_TIME > 0) delayMicroseconds(DRIVER_STEP_TIME);
            setPin(0, 0);
        } else if (*_step) {
            _step((dir > 0) ^ _globDir);
        }
    }
    uint8_t _enPin = 255;
    bool _enDir = false;
    bool _globDir = false;
    int8_t _pdir = 0;
    int8_t thisStep = 0;
    void (*_step)(uint8_t a) = NULL;
    void (*_power)(bool a) = NULL;
#ifdef __AVR__
    volatile uint8_t *_port_reg[_PINS_AMOUNT];
    volatile uint8_t _bit_mask[_PINS_AMOUNT];
#else
    uint8_t _pins[_PINS_AMOUNT];
#endif
};
#endif