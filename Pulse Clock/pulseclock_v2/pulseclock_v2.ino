/*
   Combining PulseSensor_BPM_Alternative from PulseSensor Playground and NeoPixel Ring simple sketch from Adafruit Neopixel

   Here is a link to the tutorial that discusses this code
   https://pulsesensor.com/pages/getting-advanced

   Copyright World Famous Electronics LLC - see LICENSE
   Contributors:
     Joel Murphy, https://pulsesensor.com
     Yury Gitman, https://pulsesensor.com
     Bradford Needham, @bneedhamia, https://bluepapertech.com

   PulseSensor example under the MIT License, Neopixel example under GPLv3

   This software is not intended for medical use.
*/

/*
   Every Sketch that uses the PulseSensor Playground must
   define USE_ARDUINO_INTERRUPTS before including PulseSensorPlayground.h.
   Here, #define USE_ARDUINO_INTERRUPTS false tells the library to
   not use interrupts to read data from the PulseSensor.

   If you want to use interrupts, simply change the line below
   to read:
     #define USE_ARDUINO_INTERRUPTS true

   Set US_PS_INTERRUPTS to false if either
   1) Your Arduino platform's interrupts aren't yet supported
   by PulseSensor Playground, or
   2) You don't wish to use interrupts because of the side effects.

   NOTE: if US_PS_INTERRUPTS is false, your Sketch must
   call pulse.sawNewSample() at least once every 2 milliseconds
   to accurately read the PulseSensor signal.
*/
#define USE_ARDUINO_INTERRUPTS false
#include <PulseSensorPlayground.h>

/*
   The format of our output.

   Set this to PROCESSING_VISUALIZER if you're going to run
    the Processing Visualizer Sketch.
    See https://github.com/WorldFamousElectronics/PulseSensor_Amped_Processing_Visualizer

   Set this to SERIAL_PLOTTER if you're going to run
    the Arduino IDE's Serial Plotter.
*/
const int OUTPUT_TYPE = SERIAL_PLOTTER;

/*
   Pinout:
     PULSE_INPUT = Analog Input. Connected to the pulse sensor
      purple (signal) wire.
     PULSE_BLINK = digital Output. Connected to an LED (and 1K series resistor)
      that will flash on each detected pulse.
     PULSE_FADE = digital Output. PWM pin onnected to an LED (and 1K series resistor)
      that will smoothly fade with each pulse.
      NOTE: PULSE_FADE must be a pin that supports PWM. Do not use
      pin 9 or 10, because those pins' PWM interferes with the sample timer.
     THRESHOLD should be set higher than the PulseSensor signal idles
      at when there is nothing touching it. The expected idle value
      should be 512, which is 1/2 of the ADC range. To check the idle value
      open a serial monitor and make note of the PulseSensor signal values
      with nothing touching the sensor. THRESHOLD should be a value higher
      than the range of idle noise by 25 to 50 or so. When the library
      is finding heartbeats, the value is adjusted based on the pulse signal
      waveform. THRESHOLD sets the default when there is no pulse present.
      Adjust as neccesary.
*/
const int PULSE_INPUT = A0;
const int PULSE_BLINK = LED_BUILTIN;
const int PULSE_FADE = 5;
const int THRESHOLD = 550;  // Adjust this number to avoid noise when idle

int interval = 500;  //
unsigned long passedtime;
/*
   samplesUntilReport = the number of samples remaining to read
   until we want to report a sample over the serial connection.

   We want to report a sample value over the serial port
   only once every 20 milliseconds (10 samples) to avoid
   doing Serial output faster than the Arduino can send.
*/
byte samplesUntilReport;
const byte SAMPLES_PER_SERIAL_SAMPLE = 10;

/*
   All the PulseSensor Playground functions.
*/
PulseSensorPlayground pulseSensor;


/* NeoPixel Ring simple sketch (c) 2013 Shae Erisson
 Released under the GPLv3 license to match the rest of the
 Adafruit NeoPixel library
*/


#include <Adafruit_NeoPixel.h>
#ifdef __AVR__
#include <avr/power.h>  // Required for 16 MHz Adafruit Trinket
#endif

// Which pin on the Arduino is connected to the NeoPixels?
#define PIN 6  // On Trinket or Gemma, suggest changing this to 1

// How many NeoPixels are attached to the Arduino?
#define NUMPIXELS 60  // Popular NeoPixel ring size

// When setting up the NeoPixel library, we tell it how many pixels,
// and which pin to use to send signals. Note that for older NeoPixel
// strips you might need to change the third parameter -- see the
// strandtest example for more information on possible values.
Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800);

//Set up RTC
#include <RTCZero.h>  // include library
RTCZero rtc;          // Create RTC object

#define DELAYVAL 200  // Time (in milliseconds) to pause between pixels
int red = 100;
int pulseRate;  // I'll use this to drive the clock
int ringCount = 0;
int brightness = 100;
float fadeRate = 0.035;

float currentPixel[60];

/* Change these values to set the current initial time */
const byte seconds = 0;
const byte minutes = 40;
const byte hours = 2;

/* Change these values to set the current initial date */
const byte day = 15;
const byte month = 6;
const byte year = 15;

//clock numbers
int h = 1;
int m = 15;
int s = 30;

// make connected run once with a boolean
bool alreadyConnected = false;

void setup() {

  /*
     Use 115200 baud because that's what the Processing Sketch expects to read,
     and because that speed provides about 11 bytes per millisecond.

     If we used a slower baud rate, we'd likely write bytes faster than
     they can be transmitted, which would mess up the timing
     of readSensor() calls, which would make the pulse measurement
     not work properly.
  */
  Serial.begin(115200);

  pixels.begin();  // INITIALIZE NeoPixel strip object (REQUIRED)
  pixels.clear();  // Turn all pixels off

  // Configure the PulseSensor manager.
  pulseSensor.analogInput(PULSE_INPUT);
  pulseSensor.blinkOnPulse(PULSE_BLINK);
  pulseSensor.fadeOnPulse(PULSE_FADE);

  pulseSensor.setSerial(Serial);
  pulseSensor.setOutputType(OUTPUT_TYPE);
  pulseSensor.setThreshold(THRESHOLD);

  // Skip the first SAMPLES_PER_SERIAL_SAMPLE in the loop().
  samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;

  // Now that everything is ready, start reading the PulseSensor signal.
  if (!pulseSensor.begin()) {
    /*
       PulseSensor initialization failed,
       likely because our Arduino platform interrupts
       aren't supported yet.

       If your Sketch hangs here, try changing USE_PS_INTERRUPT to false.
    */
    // for (;;) {
    //   // Flash the led to show things didn't work.
    //   digitalWrite(PULSE_BLINK, LOW);
    //   delay(50);
    //   Serial.println('!');
    //   digitalWrite(PULSE_BLINK, HIGH);
    //   delay(50);
    // }
  }
  passedtime = millis();

  //RTC Setup
  rtc.begin();  // initialize RTC

  // Set the time
  rtc.setHours(hours);
  rtc.setMinutes(minutes);
  rtc.setSeconds(seconds);

  // Set the date
  rtc.setDay(day);
  rtc.setMonth(month);
  rtc.setYear(year);

  for (int j = 0; j < NUMPIXELS; j++) {
    currentPixel[j] = 0;  // set value to zero for all pixels
  }
}



void loop() {

  //PULSE
  /*
     See if a sample is ready from the PulseSensor.

     If USE_INTERRUPTS is true, the PulseSensor Playground
     will automatically read and process samples from
     the PulseSensor.

     If USE_INTERRUPTS is false, this call to sawNewSample()
     will, if enough time has passed, read and process a
     sample (analog voltage) from the PulseSensor.
  */
  if (pulseSensor.sawNewSample()) {
    /*
       Every so often, send the latest Sample.
       We don't print every sample, because our baud rate
       won't support that much I/O.
    */
    if (--samplesUntilReport == (byte)0) {
      samplesUntilReport = SAMPLES_PER_SERIAL_SAMPLE;

      // pulseSensor.outputSample();

      /*
         At about the beginning of every heartbeat,
         report the heart rate and inter-beat-interval.
      */
      if (pulseSensor.sawStartOfBeat()) {
        pulseSensor.outputBeat();
        // Serial.println ("beat"); // print every time a heartbeat is registered
      }
    }
    if (pulseSensor.getBeatsPerMinute() > 40 && pulseSensor.getBeatsPerMinute() < 200) {  // if reading is within human limits
      
      if (alreadyConnected = false) { // only run connected if it hasn't already run, we'll turn this back off in "disconnected"
        alreadyConnected = true;
        connected();
      delay(3000);
      pixels.clear();
      pixels.show();
      }

      pulseMode();
    }

    else {

      if (alreadyConnected = true) {
        alreadyConnected = false;
        disconnected();
        pixels.clear();
        pixels.show();
      }
      pixelClock();
    }
    /*******
      Here is a good place to add code that could take up
      to a millisecond or so to run.
    *******/

    /******
     Don't add code here, because it could slow the sampling
     from the PulseSensor.
  ******/
  }
}

void pixelClock() {

  // Map h, m and s to a pixel on the ring
  h = ((rtc.getHours() % 12) * 5);
  m = (rtc.getMinutes());
  s = (rtc.getSeconds());

  // Test with Serial
  Serial.print(h);
  Serial.print(":");
  Serial.print(m);
  Serial.print(":");
  Serial.print(s);
  Serial.println();

  // Write to Pixels
  // pixels.clear();
  pixels.setPixelColor(h, pixels.Color(100, 100, 100));
  pixels.setPixelColor(m, pixels.Color(186, 144, 39));
  pixels.setPixelColor(h, pixels.Color(39, 118, 186));
  pixels.show();
}

void pulseMode() {
  pulseRate = pulseSensor.getBeatsPerMinute();  // turn the BPM into pulseRate
  interval = 60000 / pulseRate;                 // divide 60 seconds by BPM, make that the interval (delay time)
  //  Serial.println(interval);

  pulseSensor.getBeatsPerMinute();


  if (millis() - passedtime > interval) {  // if more time has passed than our most recent interval, which wil either be set by pulse sensor or our default 200ms

    // Set current array value to 100, 100, 100 (we use an array so we can independently fade every pixel below while pulsing this one)
    currentPixel[ringCount] = 200;

    // Set every pixel to current array value
    pixels.setPixelColor(ringCount, pixels.Color(currentPixel[ringCount], currentPixel[ringCount], currentPixel[ringCount]));

    ringCount++;
    ringCount = ringCount % NUMPIXELS;

    passedtime = millis();
  }

  // Fade down any pixels above zero
  for (int i = 0; i < NUMPIXELS; i++) {
    if (currentPixel[i] > 0) {
      currentPixel[i] -= fadeRate;
      pixels.setPixelColor(i, pixels.Color(currentPixel[i], currentPixel[i], currentPixel[i]));
    }
  }
  pixels.show();
}

void connected() {
  Serial.println("connected");
  pixels.clear();
  // Whole ring glows green and fades back to off
  for (int k = 0; k < NUMPIXELS; k++) {
    currentPixel[k] = 100;
    pixels.setPixelColor(k, 0, currentPixel[k], 0);
  }
  for (int l = 0; l < NUMPIXELS; l++) {
    if (currentPixel[l] > 0) {
      currentPixel[l] -= 1;
    }
  }

  pixels.show();
}

void disconnected() {
  Serial.println("disconnected");
  pixels.clear();
  // Whole ring glows green and fades back to off
  for (int m = 0; m < NUMPIXELS; m++) {
    currentPixel[m] = 100;
    pixels.setPixelColor(currentPixel[m], 0, currentPixel[m], 0);
  }
  for (int n = 0; n < NUMPIXELS; n++) {
    if (currentPixel[n] > 0) {
      currentPixel[n] -= 1;
    }
  }

  pixels.show();
}
