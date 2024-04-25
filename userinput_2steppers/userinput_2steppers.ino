#include <AccelStepper.h>

// Motor 1 Connections
const int dirPin = 2;
const int stepPin = 3;

// Motor 2 Connections
const int dirPin2 = 4;
const int stepPin2 = 5;

// two stepper motors
AccelStepper myStepper1(AccelStepper::DRIVER, stepPin, dirPin);
AccelStepper myStepper2(AccelStepper::DRIVER, stepPin2, dirPin2);

//initializing led pins
const int redLED = 13;
const int greenLED = 12;

bool LEDStates;

//initializing analog pin to read battery voltage
const int readBatt = A7;
float battVolt = 0;

//timer variables to flash leds without interupting
unsigned long prevMills = 0;

void setup() {
  Serial.println("Starting");
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

  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  Serial.begin(115200);
  Serial.println("Setup successful");
}

void loop() {

  //read battery voltage
  battVolt = (5*((float)analogRead(readBatt))/1024)*4;
  Serial.println(battVolt);

  //updating time
  unsigned long currMills = millis();

  //if a second has passed
  if(currMills - prevMills >= 1000){
    prevMills = currMills;
    LEDStates = !LEDStates;
    if(battVolt > 15){
      digitalWrite(greenLED, LEDStates);
      digitalWrite(redLED, 0);
    }else{
      digitalWrite(redLED, LEDStates);
      digitalWrite(greenLED, 0);
    }
  }
  
  if (Serial.available() > 0) {
    // Expecting input format: speed1 speed2
    float speed1 = Serial.parseFloat(); // Reads the first speed until it encounters a non-integer character
    float speed2 = Serial.parseFloat(); // Reads the next integer after the space
    Serial.print(speed1);
    Serial.println(speed2);
    
    // Apply the speeds to the stepper motors
    if (1) {
      
      speed1 = -1*(speed1/360)*6400;
      speed2 = (speed2/360)*6400;
      
      myStepper1.setSpeed(speed1);
      Serial.print("Speed1: ");
      Serial.print(speed1 + String(" | "));
      myStepper2.setSpeed(speed2);
      Serial.print(" Speed2: ");
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
  myStepper1.runSpeed();
  myStepper2.runSpeed();
}
