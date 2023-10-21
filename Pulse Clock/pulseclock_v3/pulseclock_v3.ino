/*
   Combining PulseSensor_BPM_Alternative from PulseSensor Playground and NeoPixel Ring simple sketch from Adafruit Neopixel
   More info here: https://www.notion.so/nuff/Pulse-Clock-a7fbea9ade7940f2a7756a85b0ea8c0b?pvs=4#6088fc27881b4046944c3d6e8e83dcf9

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

int interval = 500;  // Default speed for pulse clock if no good values
unsigned long passedtime;
int buttonMode = 0;  // using button to switch mode
int buttonState = 0;
int previousButton = 0;
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
int red = 0;
int pulseRate;  // I'll use this to drive the clock
int ringCount = 0;
int brightness = 100;
float fadeRate = 0.035;

float currentPixelR[60];
float currentPixelG[60];
float currentPixelB[60];

//Clock Stuff
#include <WiFiNINA.h>
#include <WiFiUdp.h>
#include <NTPClient.h>

//WiFi Credentials (370 Jay is "sandbox370", "+s0a+s03!2gether?")

#define SSID "Verizon_Q9CXS7" // WiFi Network
#define PASS "cif6-moat-movie" // WiFi Password

const long utcOffsetWinter = -18000;  // Offset from EST in seconds (3600 seconds = 5h) -- UTC-5 (EDT)
const long utcOffsetSummer = -14400;  // Offset from UTC in seconds (7200 seconds * 4h) -- UTC-4 (EST)
unsigned long lastupdate = 0UL;

// Define NTP Client to get time
WiFiUDP udpSocket;
NTPClient ntpClient(udpSocket, "pool.ntp.org", utcOffsetSummer);

/* Change these values to set the current initial time */
// const byte seconds = 0;
// const byte minutes = 40;
// const byte hours = 2;

//clock numbers
int h;
int m;
int s;

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

  // Setting up push button to switch modes until pulse sensing works
  pinMode(2, INPUT);  // set the pushbutton pin to be an input
  buttonState = digitalRead(2);


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

  //Connect to WiFiand Initialise NTP
  WiFi.begin(SSID, PASS);

  Serial.print("Connecting to ");
  Serial.println(SSID);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(250);
  }

  Serial.println(" Done!");

  ntpClient.begin();

  ntpClient.update();

  Serial.print(ntpClient.getHours());
  Serial.print(":");
  Serial.print(ntpClient.getMinutes());
  Serial.print(":");
  Serial.println(ntpClient.getSeconds());

  //RTC Setup
  rtc.begin();  // initialize RTC


  // Set the time
  rtc.setHours(ntpClient.getHours());
  rtc.setMinutes(ntpClient.getMinutes());
  rtc.setSeconds(ntpClient.getSeconds());


  for (int j = 0; j < NUMPIXELS; j++) {
    currentPixelR[j] = 0;  // set value to zero for all pixels
    currentPixelG[j] = 0;  
    currentPixelB[j] = 0;  
  }
}



void loop() {

  // buttonPressed = false;
  //     pixelClock();


  if (digitalRead(2) == HIGH && digitalRead(2) != previousButton) {
    // Serial.println("Pressed");
    previousButton = digitalRead(2);
  } else if (digitalRead(2) == LOW && digitalRead(2) != previousButton) {
    // Serial.println("Released");
    previousButton = 0;
    buttonMode++;
  }

  if (buttonMode % 2 == 1) {
    if (alreadyConnected == false) {  // only run connected if it hasn't already run, we'll turn this back off in "disconnected"

      connected();
      // delay(1000);
      // pixels.clear();
      alreadyConnected = true;
    }

    pulseMode();
  } else {
    if (alreadyConnected == true) {  // only run connected if it hasn't already run, we'll turn this back off in "disconnected"

      disconnected();
      alreadyConnected = false;
    }

    pixelClock();
  }
  //  Serial.println(pulseSensor.getBeatsPerMinute());
}



void pixelClock() {
  // Serial.println("Running pixelClock");

  // Map h, m and s to a pixel on the ring
  h = ((rtc.getHours() % 12) * 5);
  m = (rtc.getMinutes());
  s = (rtc.getSeconds());

  // Test with Serial
  // Serial.print(h);
  // Serial.print(":");
  // Serial.print(m);
  // Serial.print(":");
  // Serial.print(s);
  // Serial.println();

  // Write to Pixels
  pixels.clear();
  pixels.setPixelColor(h, pixels.Color(100, 100, 100));
  pixels.setPixelColor(h + 1, pixels.Color(100, 100, 100));
  // pixels.setPixelColor(h+2, pixels.Color(100, 100, 100));
  // pixels.setPixelColor(h+3, pixels.Color(100, 100, 100));
  pixels.setPixelColor(m, pixels.Color(186, 144, 39));
  pixels.setPixelColor(s, pixels.Color(0, 200, 0));
  pixels.show();
}

void pulseMode() {
  // Serial.println("Running pulseMode");

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

      pulseRate = pulseSensor.getBeatsPerMinute();  // turn the BPM into pulseRate
      interval = 60000 / pulseRate;                 // divide 60 seconds by BPM, make that the interval (delay time)
      // Serial.println(interval);
      // Serial.println (pulseSensor.getBeatsPerMinute());


      if (millis() - passedtime > interval) {  // if more time has passed than our most recent interval, which wil either be set by pulse sensor or our default 200ms

        // Map red amount to add based on current BPM
        red = map(pulseRate, 40, 200, -40, 40);
        
        // Set current array RGB values (we use an array so we can independently fade every pixel below while pulsing this one)
        currentPixelR[ringCount] = 100; // + red; //comment out the +red to go back to monochrome
        currentPixelG[ringCount] = 100;
        currentPixelB[ringCount] = 100;

        

        // Set every pixel to current array value
        pixels.setPixelColor(ringCount, pixels.Color(currentPixelR[ringCount], currentPixelG[ringCount], currentPixelB[ringCount]));

        ringCount++;
        ringCount = ringCount % NUMPIXELS;

        // Add 10% to each light to create a pulse
        // for (int i = 0; i < NUMPIXELS; i++){
        //   currentPixelR[i] += 2;
        //   currentPixelG[i] += 2;
        //   currentPixelB[i] += 2;
        // }

        passedtime = millis();
      }

      // Fade down any pixels above zero
      for (int i = 0; i < NUMPIXELS; i++) {
        if (currentPixelR[i] > 0) {
          currentPixelR[i] -= fadeRate; 
        }
        if (currentPixelG[i] > 0) {
          currentPixelG[i] -= fadeRate; 
        }
        if (currentPixelB[i] > 0) {
          currentPixelB[i] -= fadeRate; 
        }
        pixels.setPixelColor(i, pixels.Color(currentPixelR[i], currentPixelG[i], currentPixelB[i]));
      }
      pixels.show();
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

void connected() {
  // Serial.println("connected");
  pixels.clear();
  // Whole ring glows green and fades back to off
  
    for (int l = 0; l < NUMPIXELS; l++) {

      pixels.setPixelColor(l, 0, 99, 0);
          pixels.show();
    }


  for (int k = 99; k > 0; k--) {
    for (int l = 0; l < NUMPIXELS; l++) {

      pixels.setPixelColor(l, 0, k, 0);
    }
    pixels.show();
    delay(10);
  }
  pixels.clear();
}

//Disconnected Animation: red ring in, red ring out.
void disconnected() {
  // Serial.println("disconnected");
  pixels.clear();

      for (int l = 0; l < NUMPIXELS; l++) {

      pixels.setPixelColor(l, 99, 0, 0);
          pixels.show();
    }
delay(100);
     for (int l = 0; l < NUMPIXELS; l++) {

      pixels.setPixelColor(l, 0, 0, 0);
          pixels.show();
    }
  pixels.clear();
  delay(100);
}
