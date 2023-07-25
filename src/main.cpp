#include <Arduino.h>

#include <AccelStepper.h>

//#include <SoftwareSerial.h>

#define RX 3
#define TX 4

#define step_pin 1
#define dir_pin 0
#define drv_enable_pin 2

#define max_speed 4000
#define default_update_interval 20

//SoftwareSerial Serial1(RX, TX);
  

// move_t moves[] = {
//   { 's', 0, 24, 2400}, // s for speed notation (startSpeed, deltaSpeed, endSpeed) DELTA IS PER INTERVAL 
//   { 's', 2400, -2, 0},
//   { 'e', 0, 0, 0},
// };
// startSpeed //Not needed, just can be pulled from the stepper object
// deltaSpeed
// endSpeed

// uint32_t moveStartTime;
// int32_t delta;
// int32_t currentPos;
// int16_t moveStartSpeed;
// int16_t moveEndSpeed;
// float  moveAcceleration;

uint32_t stepIndex;
uint32_t sequenceIndex;

uint32_t scheduledTime;

uint16_t updateInterval;
uint32_t startMillis;

int32_t nextPhaseAt;

int32_t absoluteStep;
bool active;

uint8_t phase;
uint16_t accelPhases[8];
uint16_t speedPhases[8];

//ControlledStepper DisplayHand(step_pin, dir_pin);

AccelStepper Stepper(AccelStepper::DRIVER, step_pin, dir_pin);

// int16_t stepsToMove() {
  
// }
  //Calculating the number of steps needed to reach a certain speed (delta in speed)
  // int32_t stepsRequired;
  // int16_t speedDelta = moveEndSpeed - moveStartSpeed;
  // int16_t timeDelta = speedDelta/moveAcceleration; // Returns the number of seconds of acceleration to reach the requested speed
  //what if just does acceleration, and stops accelerating once at desired speed?
  //HOWTO figure out the number of steps to stop - we have the time to stop/change accel
  //steps to stop in accelstepper
  // speed^2/2a
  // int16_t stepsToAccelerate = (speedDelta * speedDelta) / 2 * moveAcceleration

void setup() {
  phase = 0;
  absoluteStep = 0;
  active = 1;
  Stepper.setCurrentPosition(0);
  pinMode(drv_enable_pin, OUTPUT);
  digitalWrite(drv_enable_pin, HIGH); //Enables DRV8834

  updateInterval = default_update_interval;

  accelPhases[0] = 1000;
  speedPhases[0] = 1600;
}

int32_t stepsRequired(int32_t speedDelta, float acceleration) { // only works when stopping or starting at 0
  int32_t result = 0;

  result = ((speedDelta * speedDelta) / (2.0 * acceleration));

  return result;

  // speed^2/2a
  // int16_t stepsToAccelerate = (speedDelta * speedDelta) / 2 * moveAcceleration
}

// int32_t reachSpeed(int16_t targetSpeed) { // Returns the number of steps to reach this speed
//   int16_t speedDelta = targetSpeed - Stepper.speed();

//   if (speedDelta > 0) {
//     uint32_t moveSteps = stepsRequired(targetSpeed);
//     Stepper.move(moveSteps*2); // There will be extra shit here?
//     return moveSteps;
//   } else if (speedDelta < 0) {
//     uint32_t moveSteps = stepsRequired(targetSpeed);
//     Stepper.stop();
//     return moveSteps;
//   } else {
//     return 0;
//   }
// }


void executeMove(int16_t a, void (*func)(int16_t)) {
  func(a);
}

void executeMove(int16_t a, int16_t b, void (*func)(int16_t, int16_t)) {
  func(a, b);
}

long getVccVoltage() {
  //This function based on http://provideyourown.com/2012/secret-arduino-voltmeter-measure-battery-voltage/ Also acceissible at https://www.instructables.com/Secret-Arduino-Voltmeter/
  // For ALL OF THIS, and in general, make sure L is appended to numbers when doing math. See: https://www.arduino.cc/reference/en/language/variables/data-types/long/
  ADMUX = _BV(MUX3) | _BV(MUX2);
  
  delay(2); // Wait for Vref to settle
  ADCSRA |= _BV(ADSC); // Start conversion
  while (bit_is_set(ADCSRA,ADSC)); // measuring

  uint8_t low  = ADCL; // must read ADCL first - it then locks ADCH  
  uint8_t high = ADCH; // unlocks both

  long result = (high<<8) | low;
  // int16_t result = (high<<8) | low;
  // I think it's a long for the math next, but also know it should fit in 16 bits?

  result = 1125300L / result; // Calculate Vcc (in mV); 1125300 = 1.1*1023*1000
  
  //Unsure if these are needed.
  // ADMUX&=~(REFS0);
  // ADMUX&=~(REFS1);
  
  return result; // Vcc in millivolts
}

// For later
/*
if (getVccVoltage() < 3700) {
  Stepper.move(-5);
}
*/
//Then just move the voltage with your supply until below 3.7. See if it's even close

void loop() {

  int32_t lastPosition = Stepper.currentPosition();

  Stepper.run();

  if (Stepper.currentPosition() != lastPosition) {
    absoluteStep++;
  }

  if (phase == 0) {
    // Serial1.println("in phase 0");
    Stepper.setAcceleration(accelPhases[phase]);
    Stepper.setMaxSpeed(speedPhases[phase]);
    
    int32_t stepsToAccel = stepsRequired(speedPhases[phase], accelPhases[phase]);
    int32_t stepsToDeccel = stepsRequired(1600, 500);
    // Serial1.println(stepsToAccel);

    nextPhaseAt = stepsToAccel;

    Stepper.move(stepsToAccel + stepsToDeccel + 1000);
    phase++;

  } else if (absoluteStep >= nextPhaseAt && phase == 1) {
    // Serial1.println("in phase 1");
    Stepper.setAcceleration(500);

    nextPhaseAt = 60000;
    phase++;
  } else if (absoluteStep >= nextPhaseAt && phase == 2) {
    // Serial1.println("in phase 2");
    active = 0;
  }

  
  // ControlledStepper.runSpeed()

  // if (currentSpeed < moveEndSpeed) {  //test if more speed is needed
  //   //Will need to differentiate the direction of speed change (e.g. if decelerating?)
  //   int32_t elapsedTime = millis() - moveStartTime;
  //   currentSpeed = elapsedTime/1000 * moveAcceleration;
  // } else {
  //   currentSpeed = currentSpeed;
  // }
  
  //DisplayHand.setSpeed(currentSpeed) 
  //DisplayHand.runSpeed()


  // if (scheduledTime <= micros()) {
  //   scheduledTime += updateInterval;

  //   switch (moves[sequenceIndex].type) {
  //     case 's':
  //       if (displayHand.speed() != moves[sequenceIndex].end) {  // Delta is positive
  //         uint16_t elapsedMillis = millis() - startMillis

  //         DisplayHand.setSpeed(moves[sequenceIndex].start + moves[sequenceIndex].delta);
  //       } else {
  //         sequenceIndex++;
  //       }
  //       break;

  //     case 'e':
  //       digitalWrite(drv_enable_pin, moves[sequenceIndex].delta);
  //       break;

  //     default:
  //       break;
  //   }
  // }
}

// https://www.dummies.com/article/academics-the-arts/science/physics/how-to-calculate-time-and-distance-from-acceleration-and-velocity-174278/

