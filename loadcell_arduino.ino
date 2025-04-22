#include "HX711.h"

// Download the HX711 library in the arduino library to get use the
// load cell booster

// HX711 circuit wiring
const int LOADCELL_DOUT_PIN = 2;
const int LOADCELL_SCK_PIN = 3;

HX711 scale;

void setup() {
  Serial.begin(115200);
  scale.begin(LOADCELL_DOUT_PIN, LOADCELL_SCK_PIN);

  Serial.println("Initializing the scale...");
  
  // Wait for scale to be ready
  while (!scale.is_ready()) {
    Serial.println("Waiting for the HX711...");
    delay(500);
  }

  // Tare to zero the scale (remove any load)
  scale.tare();
  Serial.println("Scale tared.");

  // Set your own calibration factor here
  // current_factor * (actual weight / a bunch of factors)
  scale.set_scale(2280.0*(4980/1800)/5*.97);
}

void loop() {
  // Only print if scale is ready
  if (scale.is_ready()) {
    float weight = scale.get_units(1); // Average of 5 readings
    Serial.print("Weight: ");
    Serial.print(weight, 3);  // Show two decimal places
    Serial.println(" grams");
  } else {
    Serial.println("HX711 not found.");
  }

  delay(0.001);
}
