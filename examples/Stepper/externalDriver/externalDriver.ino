#define PIN_A 2
#define PIN_B 4
#define PIN_C 3
#define PIN_D 5
#include <GyverStepper.h>
GStepper<STEPPER4WIRE, STEPPER_VIRTUAL> stepper(2048);
void setup() {
  Serial.begin(9600);
  pinMode(PIN_A, 1);
  pinMode(PIN_B, 1);
  pinMode(PIN_C, 1);
  pinMode(PIN_D, 1);
  stepper.setRunMode(KEEP_SPEED);
  stepper.setSpeedDeg(100);
  stepper.setAcceleration(500);
  stepper.attachStep(step);
  stepper.attachPower(pwr);
  stepper.autoPower(1);
}
void step(byte val) {
  digitalWrite(PIN_D, val & 1);
  val >>= 1;
  digitalWrite(PIN_C, val & 1);
  val >>= 1;
  digitalWrite(PIN_B, val & 1);
  val >>= 1;
  digitalWrite(PIN_A, val & 1);
}
void pwr(bool val) {
  Serial.println(val);
}
void loop() {
  stepper.tick();
  static uint32_t tmr;
  if (millis() - tmr >= 3000) {
    tmr = millis();
    static bool dir = 1;
    dir = !dir;
    if (!dir) stepper.stop();
    else stepper.setSpeedDeg(100);
  }
}