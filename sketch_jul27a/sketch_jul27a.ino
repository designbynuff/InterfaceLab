int pot1;

void setup() {
  Serial.begin(9600);
  pinMode(2, INPUT);
}

void loop() {
  pot1 = analogRead(A0);


  if ( Serial.available() > 0 ) {
    int incoming = Serial.read();
    Serial.print(pot1);

  }
}