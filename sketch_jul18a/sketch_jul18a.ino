// Interface Lab July 18
// Switching between 3 LEDs with a push button (traffic lights)

int prevButtonState = 0;  // variable for previous button state
int counter = 0;          // counter to switch states

void setup() {
  // put your setup code here, to run once:

  //Begin the serial port so we can communicate/debug
  Serial.begin(9600);

  //set D2 to be an input (button) (note that digital pins don't need to be declared as "D2"—Arduino assumes digital by default)
  pinMode(2, INPUT);

  // D3-D5 as outputs (LEDs)
  pinMode(3, OUTPUT);  // white
  pinMode(4, OUTPUT);  // yellow
  pinMode(5, OUTPUT);  // green
}

void loop() {
  // put your main code here, to run repeatedly:

  int buttonState = digitalRead(2);  // read current button state (1 is pressed, 0 is not)
  Serial.println(buttonState);       // print to serial

  if (buttonState == 1 && prevButtonState == 0) {  // press down state
    prevButtonState = 1;
  }

  if (buttonState == 0 && prevButtonState == 1) {  // release
    Serial.println("Button Released, update the counter");
    counter = counter + 1;  // add 1 to the counter

    prevButtonState = 0;  // update the "previous" button state
    delay(5);
  }

  if (counter == 0) {
    digitalWrite(3, LOW);  // white
    digitalWrite(4, LOW);  // yellow
    digitalWrite(5, LOW);  // green
  }
  if (counter == 1) {
    digitalWrite(3, HIGH);  // white
    digitalWrite(4, LOW);   // yellow
    digitalWrite(5, LOW);   // green
  }

  if (counter == 2) {
    digitalWrite(3, LOW);   // white
    digitalWrite(4, HIGH);  // yellow
    digitalWrite(5, LOW);   // green
  }

  if (counter == 3) {
    digitalWrite(3, LOW);   // white
    digitalWrite(4, LOW);   // yellow
    digitalWrite(5, HIGH);  // green
  }

  if (counter > 3) {  //reset the counter with if statement
    counter = 0;
  }

  //counter = counter % 4; // uncomment to reset the counter using modulo
}

