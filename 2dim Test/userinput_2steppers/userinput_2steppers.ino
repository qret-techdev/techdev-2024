#include <AccelStepper.h>

// Motor 1 Connections
const int dirPin = 4;
const int stepPin = 3;

// Motor 2 Connections
const int dirPin2 = 6;
const int stepPin2 = 5;

// two stepper motors
AccelStepper myStepper1(AccelStepper::DRIVER, stepPin, dirPin);
AccelStepper myStepper2(AccelStepper::DRIVER, stepPin2, dirPin2);

void setup() {
  // Motor 1 Configuration
  myStepper1.setMaxSpeed(13000); // Set max speed
  myStepper1.setSpeed(0);    // Set initial speed
  
  // Motor 2 Configuration
  myStepper2.setMaxSpeed(13000); //max speed
  myStepper2.setSpeed(0);    // initial speed
  
  // Microstepping pins configuration
  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);

  // Microstepping for 1/32
  digitalWrite(A0, HIGH);
  digitalWrite(A1, HIGH);
  digitalWrite(A2, HIGH);

  Serial.begin(115200);
}

void loop() {
  if (Serial.available() > 0) {
    // Expecting input format: speed1 speed2
    String speeds = Serial.readStringUntil('\n');
    
    // split the string into two substrings
    int spaceIndex = speeds.indexOf(' ');
    String speed1_str = speeds.substring(0, spaceIndex);
    String speed2_str = speeds.substring(spaceIndex + 1);

    // convert strings to float
    float speed1 = speed1_str.toFloat();
    float speed2 = speed2_str.toFloat();
    
    // Apply the speeds to the stepper motors
    if (1) {
      
      speed1 = (speed1/360)*6400;
      speed2 = (speed2/360)*6400;
      
      myStepper1.setSpeed(speed1);
      Serial.print("Speed1 set to: ");
      Serial.print(speed1);
      myStepper2.setSpeed(speed2);
      Serial.print(" Speed2 set to: ");
      Serial.println(speed2);
    } else {
      Serial.println("Invalid input. Enter positive numbers separated by a space.");
    }

    // Clear the serial buffer by reading until newline or timeout
    while (Serial.available() > 0) {
      char c = Serial.read();
      if (c == '\n' || c == '\r') break;
    }
  }

  // Run both stepper motors at their set speeds

  myStepper1.runSpeed();
  myStepper2.runSpeed();
}
