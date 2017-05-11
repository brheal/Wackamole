
/******************************************************************************
  Force_Sensitive_Resistor_Example.ino
  Example sketch for SparkFun's force sensitive resistors
  (https://www.sparkfun.com/products/9375)
  Jim Lindblom @ SparkFun Electronics
  April 28, 2016

  Create a voltage divider circuit combining an FSR with a 3.3k resistor.
  - The resistor should connect from A0 to GND.
  - The FSR should connect from A0 to 3.3V
  As the resistance of the FSR decreases (meaning an increase in pressure), the
  voltage at A0 should increase.

  Development environment specifics:
  Arduino 1.6.7
******************************************************************************/
#include<FastLED.h>
#define NUM_LEDS 40
const int FSR_PIN = A0; // Pin connected to FSR/resistor divider

// Measure the voltage at 5V and resistance of your 3.3k resistor, and enter
// their value's below:
const float VCC = 4.98; // Measured voltage of Ardunio 5V line
const float R_DIV = 3230.0; // Measured resistance of 3.3k resistor
CRGBArray<NUM_LEDS> leds;
const int delayAfterForceInSeconds = 5;


void setup()
{
  Serial.begin(9600);
  FastLED.addLeds<NEOPIXEL, 7>(leds, NUM_LEDS);
  pinMode(FSR_PIN, INPUT);
}

void loop()
{
  int fsrADC = analogRead(FSR_PIN);
  // If the FSR has no pressure, the resistance will be
  // near infinite. So the voltage should be near 0.
  if (fsrADC > 30) // If the analog reading is non-zero
  {
    // Use ADC reading to calculate voltage:
    float fsrV = fsrADC * VCC / 1023.0;
    // Use voltage and static resistor value to
    // calculate FSR resistance:
    float fsrR = R_DIV * (VCC / fsrV - 1.0);
    Serial.println("Resistance: " + String(fsrR) + " ohms");
    // Guesstimate force based on slopes in figure 3 of
    // FSR datasheet:
    float force;
    float fsrG = 1.0 / fsrR; // Calculate conductance
    // Break parabolic curve down into two linear slopes:
    if (fsrR <= 600)
      force = (fsrG - 0.00075) / 0.00000032639;
    else
      force =  fsrG / 0.000000642857;
    Serial.println("Force: " + String(force) + " g");

    Serial.println();

    if (force < 200) {
      leds.fill_solid(CRGB::Black);
    }

    int threshold = 200;
    for (int i = 0 ; i < NUM_LEDS; i++) {
      threshold = threshold + 125; //
      if (force > threshold) {
        if (i > 35) {
          leds[i] = CRGB::Blue;
        }
        else if (i > 23) {
          leds[i] = CRGB::Green;
        } else {
          leds[i] = CRGB::Red;
        }
      } else {
        leds[i] = CRGB::Black;
      }

    }

    FastLED.show();

    //wait so we can see results for longer
    Serial.println("showing results...");
    delay(delayAfterForceInSeconds * 1000);
    Serial.println("done showing");
    //clear leds
    leds.fill_solid(CRGB::Black);
    FastLED.show();
    Serial.println("reset leds");
  }

  delay(100);
}
