int pot1;
int pot2;
int button;

void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT);
}

void loop() {
  pot1 = analogRead(A0);
  pot2 = analogRead(A1);
  button = digitalRead(2);

  if ( Serial.available() > 0 ) {
    int incoming = Serial.read();
    Serial.print(pot1);
    Serial.print(",");
    Serial.print(pot2);
    Serial.print(",");
    Serial.println(button);
  }
}