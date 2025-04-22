#include "HX711.h"

// Download HX711 library in arduino IDE to use the 
// load cell booster

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

HX711 scale;

void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);
  // Tare to zero the scale (remove any load)
  scale.tare();
  Serial.println("Scale tared.");
  // Set your own calibration factor here
  // current_factor * (actual weight / a bunch of factors)
  scale.set_scale(2280.0*(4980/1800)/5*.97);
}

void loop() {
  float weight = scale.get_units(1); // Average of 1 readings
  Serial.print("Weight: ");
  Serial.print(weight, 3);  // Show three decimal places
  Serial.println(" grams");
  delay(0.001);
}
