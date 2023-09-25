#include <FastLED.h>
#include <SevenSeg.h>
#include <VescUart.h>
#include <SimpleKalmanFilter.h>

#define DATA_PIN    5
#define NUM_LEDS    10
#define BRIGHTNESS  64
#define LED_TYPE    WS2812B
#define COLOR_ORDER GRB

const int buttonPin = 4; // the pin number of the button
const int ledPin = 13;   // the pin number of the LED
bool ledState = false;   // current state of the LED
const unsigned long ledUpdateInterval = 500; // interval in milliseconds between LED strip updates
unsigned long lastLedUpdate = 0; // variable to store the last time the LED strip was updated
unsigned long previousMillis = 0;
const unsigned long interval = 1000;

/** Initiate VescUart class */
VescUart UART;

int rpm;
int velocity;
float voltage;
float batpercentage;

// Define the array of leds
CRGB leds[NUM_LEDS];
SevenSeg disp(6,7,8,9,10,11,12);
const int numOfDigits=2;
int digitPins[numOfDigits]={3,2};

#define UPDATES_PER_SECOND 100

SimpleKalmanFilter KF_velocity(2, 2, 0.01);

void setup() {
    Serial.begin(115200);
    while (!Serial) { ; }

    UART.setSerialPort(&Serial);

    unsigned long currentMillis = millis();
    if (currentMillis - previousMillis >= interval) {
        previousMillis = currentMillis;
    }

    //setup led
    FastLED.addLeds<LED_TYPE, DATA_PIN, COLOR_ORDER>(leds, NUM_LEDS);
    FastLED.setBrightness(BRIGHTNESS);
    disp.setDigitPins(numOfDigits, digitPins);

    pinMode(buttonPin, INPUT_PULLUP);
  pinMode(ledPin, OUTPUT);
}

void loop() {
   ///////// Read values //////////
    if (UART.getVescValues()) {
        rpm = (UART.data.rpm) / 20;  // The '20' is the number of pole pairs in the motor.
        voltage = (UART.data.inpVoltage);
        int rpm_filtered = KF_velocity.updateEstimate(rpm);
        velocity = (rpm_filtered * 3.142 * 0.228 * 60) / 1600;
        batpercentage = ((voltage - 34) / 7) *100;
    }

    disp.write(velocity);

    // Update LED strip only if ledUpdateInterval milliseconds have passed since the last update
  unsigned long currentMillis = millis();
  if (currentMillis - lastLedUpdate >= ledUpdateInterval) {
    lastLedUpdate = currentMillis;

    // Map the battery percentage to a range from 0 to 5
    int battery_level = map(batpercentage, 0, 100, 0, 5);

    // Set the appropriate number of LEDs to light up based on the battery level
    for (int i = 0; i < NUM_LEDS; i++) {
      if (i < 3) {
        leds[i] = CRGB::Black;  // turn off LED 0-2
      } else if (i == 3) {
        if (batpercentage < 10) {
          leds[i] = CRGB::Red;  // turn on red LED for battery level below 10%
        } else {
          leds[i] = CRGB::Black;  // turn off LED 3 for battery level above 10%
        }
      } else if (i < battery_level + 4) {
        leds[i] = CRGB::Aqua;  // turn on green LED for battery level
      } else {
        leds[i] = CRGB::Black;  // turn off LED if battery level is below LED position
      }
    }

    FastLED.show();
  }
}