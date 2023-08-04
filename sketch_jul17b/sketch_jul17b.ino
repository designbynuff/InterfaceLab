int peakValue = 0;
int threshold = 50;   //set your own value based on your sensors
int noise = 5;        //set a noise value based on your particular sensor
 
void setup() {
  Serial.begin(9600);
}
 
void loop() {
  //read sensor on pin A0:
  int sensorValue = analogRead(A0);
  // check if it's higher than the current peak:
  if (sensorValue > peakValue) {
    peakValue = sensorValue;
  }
  if (sensorValue <= threshold - noise ) {
    if (peakValue > threshold + noise) {
      // you have a peak value:
      Serial.println(peakValue);
      // reset the peak value:
      peakValue = 0;
    }
  }
}