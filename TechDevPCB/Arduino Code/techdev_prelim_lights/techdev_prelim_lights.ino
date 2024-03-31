//initializing led pins
const int redLED = 13;
const int greenLED = 12;

bool LEDStates;

//initializing analog pin to read battery voltage
const int readBatt = A7;
unsigned long battVolt = 0;
const int battState = 1; //0 for red, 1 for green

//timer variables to flash leds without interupting
unsigned long prevMills = 0;

void setup() {
  pinMode(redLED, OUTPUT);
  pinMode(greenLED, OUTPUT);

  Serial.begin(115200);
}

void loop() {

  //read battery voltage
  battVolt = (5*analogRead(readBatt)/1024)*4;

  //updating time
  unsigned long currMills = millis();

  //if a second has passed
  if(currMills - prevMills >= 1000){
    prevMills = currMills;
    LEDStates = !LEDStates;
    if(battVolt > 15){
      digitalWrite(greenLED, LEDStates);
    }else{
      digitalWrite(redLED, LEDStates);
    }
  }

}
