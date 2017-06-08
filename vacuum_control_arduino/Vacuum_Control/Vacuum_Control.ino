int handshake = 0;
int controlPin = 17;

void setup() {
  pinMode(controlPin, OUTPUT);
  Serial.begin(115200);
}

void loop() {
  if (Serial.available() > 0) {
    handshake = Serial.read();
//    Serial.println(handshake); // prints the input key DEC code

    if(handshake == 118) { // ASCII code: 118 == v (suction on in Python)
      Serial.println("suction on");
      digitalWrite(controlPin, HIGH);
    }

    if(handshake == 120) { // ASCII code: 120 == x (suction off in Python)
      Serial.println("suction off");
      digitalWrite(controlPin, LOW); 
    } 
  }
}
