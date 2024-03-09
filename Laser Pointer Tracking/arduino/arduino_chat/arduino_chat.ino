const int led_red = 2;
const int led_green = 6;

float speed = 0;

void setup() {
  Serial.begin(115200); // Initialize serial communication

  pinMode(led_red, OUTPUT);
  pinMode(led_green, OUTPUT);

  writeboth(LOW);
}

void loop() {
  if (Serial.available() > 0) {
    speed = Serial.parseFloat(); // Read speed from serial
    writeboth(LOW);
  }

  if(speed > 0){
    writeboth(LOW);
    digitalWrite(led_green, HIGH);
  }else if(speed < 0){
    writeboth(LOW);
    digitalWrite(led_red, HIGH);
  }
  
}

void writeboth(bool sig){
  digitalWrite(led_red, sig);
  digitalWrite(led_green, sig);
}
