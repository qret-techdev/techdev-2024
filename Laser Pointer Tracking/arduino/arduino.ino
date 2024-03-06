const int ledPin1 =  7;
const int ledPin2 = 3;

uint8_t a;

void setup() {
  pinMode(ledPin1, OUTPUT);
  pinMode(ledPin2, OUTPUT);
  Serial.begin(115200);
  digitalWrite(ledPin1, LOW);
}

void loop() {

  if(Serial.available() > 0){
    a = Serial.read();
  }
  
  if(a == 45){
    digitalWrite(ledPin1, HIGH);
    delay(50);
  }else if(a == 90){
    digitalWrite(ledPin2, HIGH);
  }

  digitalWrite(ledPin1, LOW);
  digitalWrite(ledPin2, LOW);

}
