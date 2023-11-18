#include <AccelStepper.h>

//
// Motor Connections (constant current, step/direction bipolar motor driver)
const int dirPin = 4;
const int stepPin = 3;





AccelStepper myStepper(AccelStepper::DRIVER, stepPin, dirPin);           // works for a4988 (Bipolar, constant current, step/direction driver)

void setup()
{  
   myStepper.setMaxSpeed(3000);   // this limits the value of setSpeed(). Raise it if you like.
   myStepper.setSpeed(2500);     // runSpeed() will run the motor at this speed - set it to whatever you like.

pinMode(A0, OUTPUT);
pinMode(A1, OUTPUT);
pinMode(A2, OUTPUT);

digitalWrite(A0, HIGH);
digitalWrite(A1, HIGH);
digitalWrite(A2, LOW);

Serial.begin(9600);
}


void loop()
{  
//  if (Serial.available() > 0) {
//    int speedValue = Serial.parseInt(); // Read the integer value from serial input
//
//    // Check if a valid number is received
//    if (speedValue > 0) {
//      myStepper.setSpeed(speedValue);
//      Serial.print("Speed set to: ");
//      Serial.println(speedValue);
//    } else {
//      Serial.println("Invalid input. Please enter a positive number.");
//    }
//
//    // Wait for a brief moment to avoid continuously reading from Serial
//    delay(10);
//  }
//
//  myStepper.runSpeed();


if (Serial.available() > 0) {
    int speedValue = Serial.parseInt(); // Read the integer value from serial input

    // Check if a valid number is received
    if (speedValue > 0) {
      myStepper.setSpeed(speedValue);
      Serial.print("Speed set to: ");
      Serial.println(speedValue);
    } else {
      Serial.println("Invalid input. Please enter a positive number.");
    }

    // Clear the serial input buffer to avoid any leftover characters
    while (Serial.available() > 0) {
      Serial.read();
    }
  }

  myStepper.runSpeed();
}
