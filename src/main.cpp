#include <Arduino.h>

#include <AccelStepper.h>

//#include <SoftwareSerial.h>

#define RX 3
#define TX 4

#define step_pin 1
#define dir_pin 0
#define drv_enable_pin 2

#define max_step_rate 4000
#define default_update_interval 20

//SoftwareSerial Serial1(RX, TX);

AccelStepper Stepper(AccelStepper::DRIVER, step_pin, dir_pin);

void setup() {
  Stepper.setCurrentPosition(0);
  pinMode(drv_enable_pin, OUTPUT);
  digitalWrite(drv_enable_pin, HIGH); //Enables DRV8834

  Stepper.setMaxSpeed(max_step_rate);
  Stepper.setAcceleration(50);
  Stepper.move(30000);
}

int32_t stepsRequired(int32_t speedDelta, float acceleration) { // only works when stopping or starting at 0
  int32_t result = 0;

  result = ((speedDelta * speedDelta) / (2.0 * acceleration));

  return result;

  // speed^2/2a
  // int16_t stepsToAccelerate = (speedDelta * speedDelta) / 2 * moveAcceleration
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

void loop() {
  Stepper.run();
}

// https://www.dummies.com/article/academics-the-arts/science/physics/how-to-calculate-time-and-distance-from-acceleration-and-velocity-174278/

