#define LIMSW_X 6
#define LIMSW_Y 7
#include "GyverPlanner.h"
Stepper<STEPPER2WIRE> stepper1(2, 5);
Stepper<STEPPER2WIRE> stepper2(3, 6);
GPlanner<STEPPER2WIRE, 2> planner;
void setup() {
  planner.addStepper(0, stepper1);
  planner.addStepper(1, stepper2);
  planner.setAcceleration(100);
  planner.setMaxSpeed(300);
  pinMode(LIMSW_X, INPUT_PULLUP);
  pinMode(LIMSW_Y, INPUT_PULLUP);
}
void loop() {
}
void homing() {
  if (digitalRead(LIMSW_X)) {
    planner.setSpeed(0, -10);
    while (digitalRead(LIMSW_X)) {
      planner.tick();
    }
    planner.brake();
  }
  if (digitalRead(LIMSW_Y)) {
    planner.setSpeed(1, -10);
    while (digitalRead(LIMSW_Y)) {
      planner.tick();
    }
    planner.brake();
  }
  planner.reset();
}