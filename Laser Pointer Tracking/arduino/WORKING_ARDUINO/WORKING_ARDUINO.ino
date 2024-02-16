#include <AccelStepper.h>

//
// Motor Connections (constant current, step/direction bipolar motor driver)
const int dirPin = 4;
const int stepPin = 3;

AccelStepper myStepper(AccelStepper::DRIVER, stepPin, dirPin);           // works for a4988 (Bipolar, constant current, step/direction driver)

void setup()
{  
   myStepper.setMaxSpeed(3000);   // this limits the value of setSpeed(). Raise it if you like.
   myStepper.setSpeed(0);     // runSpeed() will run the motor at this speed - set it to whatever you like.

//microstepping
pinMode(A0, OUTPUT);
pinMode(A1, OUTPUT);
pinMode(A2, OUTPUT);

digitalWrite(A0, HIGH);
digitalWrite(A1, HIGH);
digitalWrite(A2, LOW);

Serial.begin(115200);
}


void loop()
{  

if (Serial.available() > 0) {
    float speedValue = -1*Serial.parseFloat(); // Read the integer value from serial input
    speedValue = (speedValue/360)*1600;

    myStepper.setSpeed(speedValue);
    Serial.println(speedValue);

    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  myStepper.runSpeed();
}
