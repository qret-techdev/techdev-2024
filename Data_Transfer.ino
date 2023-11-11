uint8_t a;
uint8_t b;

#define FOV 60

#include <Adafruit_DotStar.h>
Adafruit_DotStar led(DOTSTAR_NUM, PIN_DOTSTAR_DATA, PIN_DOTSTAR_CLK, DOTSTAR_BRG);

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  led.begin();
  led.setBrightness(80);
  led.show();
}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.print(Serial.available());
  //*
  /*
  while (Serial.available() > 0){
    Serial.read();
  }
  */

  if (Serial.available() == 2){
    //Read Data
    a = Serial.read();
    b = Serial.read();
    
    //Set to green (0 = green for some reason)
    if (a==97||b==98){
      led.setPixelColor(0, led.gamma32(led.ColorHSV(0)));
    } else {
      //43690 = blue (idkkkkkk)
      led.setPixelColor(0, led.gamma32(led.ColorHSV(43690)));
      //DO NOT UNCOMMENT UNLESS YOU WANT TO FORCE A BLACKOUT
      //Serial.write(a);
      //Serial.write(b);
    }

    //print status
    Serial.write("1");
    //clear the value we just printed
    while (Serial.available() > 0){
      Serial.read();
    }
  } else if (Serial.available() > 0){
    //delayMicroseconds(1);
    if (Serial.available() == 2){
      Serial.write("1");
    } else {
      //21845 = red (wtf)
      led.setPixelColor(0, led.gamma32(led.ColorHSV(21845)));

      //print bad status, clear
      Serial.write("0");
      while (Serial.available() > 0){
        Serial.read();
      }
    }
  }
  led.show();
  //delayMicroseconds(1);
}
