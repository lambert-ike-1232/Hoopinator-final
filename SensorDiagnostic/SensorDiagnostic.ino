/*
 * QTR Sensor Diagnostic Tool
 * 
 * This program reads raw sensor values and prints them to Serial Monitor
 * Use this to verify your sensors are working before calibration
 * 
 * What to expect:
 * - Over WHITE surface: Low values (0-300)
 * - Over BLACK line: High values (500-2500+)
 */

#include <QTRSensors.h>

QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];

void setup() {
  Serial.begin(115200);

  // Setup QTRX line sensors
  qtr.setTypeRC(); // QTRX uses RC (digital) mode

  // QTRX Sensor Array pin configuration for TI-RSLK / A-Star
  // Sensors 0-7 connect to these A-Star pins:
  qtr.setSensorPins((const uint8_t[]){7, 18, 23, 20, 21, 22, 8, 6}, SensorCount);

  // QTRX has TWO emitter control pins
  // CTRL EVEN (pin 5) controls sensors 0, 2, 4, 6
  // CTRL ODD (pin 4) controls sensors 1, 3, 5, 7
  qtr.setEmitterPins(5, 4); // (CTRL_EVEN, CTRL_ODD)

  pinMode(LED_BUILTIN, OUTPUT);

  Serial.println("=== QTRX SENSOR DIAGNOSTIC ===");
  Serial.println("IMPORTANT: Remove BUZZER jumper on A-Star!");
  Serial.println("Reading raw sensor values...");
  Serial.println("Expected: LOW values on white, HIGH values on black");
  Serial.println();
  delay(2000);
}

void loop() {
  // Blink LED to show we're running
  digitalWrite(LED_BUILTIN, HIGH);
  
  // Read sensors WITHOUT calibration
  qtr.read(sensorValues);
  
  digitalWrite(LED_BUILTIN, LOW);
  
  // Print all 8 sensor values
  Serial.print("Sensors: [");
  for (int i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    if (i < SensorCount - 1) {
      Serial.print(", ");
    }
  }
  Serial.print("]");
  
  // Check if ANY sensor sees something
  int maxValue = 0;
  for (int i = 0; i < SensorCount; i++) {
    if (sensorValues[i] > maxValue) {
      maxValue = sensorValues[i];
    }
  }
  
  if (maxValue > 500) {
    Serial.print(" >>> LINE DETECTED! Max: ");
    Serial.println(maxValue);
  } else if (maxValue > 100) {
    Serial.print(" >>> Weak signal. Max: ");
    Serial.println(maxValue);
  } else {
    Serial.println(" >>> NO LINE (all sensors low)");
  }
  
  delay(200); // Update 5 times per second
}

