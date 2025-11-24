/*
 * Standalone Line Following Test
 *
 * This program tests ONLY the line following functionality in isolation.
 * It will:
 * 1. Calibrate the QTR sensors on startup
 * 2. Follow the line using proportional control
 * 3. Detect crosses and count them
 * 4. Stop at the second cross
 * 5. Send sensor data over serial for monitoring
 *
 * Based on working LineFollowArduinoSide.ino code
 * Upload this to test line following without needing the full system.
 */

#include <QTRSensors.h>
#include <AStar32U4Motors.h>

AStar32U4Motors m;
QTRSensors qtr;

// QTR Sensor configuration
const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
uint16_t linePosition = 3500;

// Motor variables
int leftMotor = 0;
int rightMotor = 0;

// Line following parameters
#define LF_BASE_SPEED 150        // Base speed for both motors
#define LF_KP 0.05               // Proportional gain for line following
#define CROSS_COOLDOWN_MS 1000   // Minimum time between cross detections (ms)

// State tracking
int isCross = 0;
int crossesDetected = 0;
unsigned long lastCrossTime = 0;
bool lineFollowingComplete = false;
bool hasSeenFirstCross = false;

// ========== SENSOR CALIBRATION ==========

void calibrateSensors() {
  // THE SENSORS ONLY CALIBRATE WHEN YOU UPLOAD NEW ARDUINO CODE TO THE ASTAR
  // After that the sensors STAY calibrated as long as the Astar has power

  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode
                                   // while calibrating, move the sensor over the line a couple times

  Serial.println("Calibrating... Move sensor over line for 10 seconds.");

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++) {
    qtr.calibrate();
  }

  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration
  Serial.println("Calibration complete!");
}

// ========== SENSOR READING & PROCESSING ==========

void readAndProcessLineSensors() {
  // Read the line sensor array
  linePosition = qtr.readLineBlack(sensorValues);
  
  // Filter out noisy readings under 300
  for (int i = 0; i <= 7; i++) {
    if (sensorValues[i] < 300) {
      sensorValues[i] = 0;
    }
  }
  
  // If all sensors are zero, set linePosition to 0
  if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && 
      sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && 
      sensorValues[6] == 0 && sensorValues[7] == 0) {
    linePosition = 0;
  }
  
  // Handle edge cases for sensors 0 and 7
  if (sensorValues[0] > 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && 
      sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && 
      sensorValues[6] == 0 && sensorValues[7] == 0) {
    linePosition = 1000;
  }
  if (sensorValues[7] > 0 && sensorValues[0] == 0 && sensorValues[1] == 0 && 
      sensorValues[2] == 0 && sensorValues[3] == 0 && sensorValues[4] == 0 && 
      sensorValues[5] == 0 && sensorValues[6] == 0) {
    linePosition = 5000;
  }
  
  // Hard cap linePosition between 1000 and 5000
  if (linePosition > 5000) {
    linePosition = 5000;
  }
  if (linePosition < 1000 && linePosition > 0) {
    linePosition = 1000;
  }
  
  // No line detected
  if (sensorValues[0] == 0 && sensorValues[1] == 0 && sensorValues[2] == 0 && 
      sensorValues[3] == 0 && sensorValues[4] == 0 && sensorValues[5] == 0 && 
      sensorValues[6] == 0 && sensorValues[7] == 0) {
    linePosition = 9500;
  }
  
  // Detect crosses: both leftmost and rightmost sensors see the line
  if ((sensorValues[7] > 500 && sensorValues[0] > 500) || 
      (sensorValues[2] > 500 && sensorValues[5] > 500)) {
    isCross = 1;
  } else {
    isCross = 0;
  }
}

// ========== LINE FOLLOWING LOGIC ==========

void executeLineFollowing() {
  // Read line sensors
  readAndProcessLineSensors();

  unsigned long currentMillis = millis();

  // Detect crosses with cooldown to avoid double-counting
  if (isCross == 1 && (currentMillis - lastCrossTime) > CROSS_COOLDOWN_MS) {
    crossesDetected++;
    lastCrossTime = currentMillis;
    hasSeenFirstCross = true;

    Serial.print(">>> CROSS DETECTED! Total crosses: ");
    Serial.println(crossesDetected);

    // If we've reached the second cross, stop
    if (crossesDetected >= 2) {
      leftMotor = 0;
      rightMotor = 0;
      lineFollowingComplete = true;
      Serial.println(">>> LINE FOLLOWING COMPLETE! Stopped at second cross.");
      return;
    }
  }

  // Don't start following until we're past the first cross
  // This prevents false starts if robot is placed on a cross
  if (!hasSeenFirstCross) {
    // Just drive forward slowly until we see the first cross
    leftMotor = 100;
    rightMotor = 100;
    return;
  }

  // Line following proportional control
  // linePosition ranges from 1000 (far left) to 5000 (far right)
  // Center is at 3000
  int error = linePosition - 3000;

  // Calculate motor speeds using proportional control
  float correction = LF_KP * error;

  leftMotor = LF_BASE_SPEED - correction;
  rightMotor = LF_BASE_SPEED + correction;

  // Constrain motor speeds to valid range
  leftMotor = constrain(leftMotor, -400, 400);
  rightMotor = constrain(rightMotor, -400, 400);
}

// ========== MOTOR CONTROL ==========

void commandMotors() {
  m.setM1Speed(rightMotor);
  m.setM2Speed(leftMotor);
}

void stopMotors() {
  leftMotor = 0;
  rightMotor = 0;
  commandMotors();
}

// ========== SERIAL REPORTING ==========

void publishSensorData() {
  // Format: linePosition,isCross,crossesDetected,leftMotor,rightMotor,sensorValues[0-7]
  Serial.print("LP:");
  Serial.print(linePosition);
  Serial.print(" | Cross:");
  Serial.print(isCross);
  Serial.print(" | Count:");
  Serial.print(crossesDetected);
  Serial.print(" | L:");
  Serial.print(leftMotor);
  Serial.print(" | R:");
  Serial.print(rightMotor);
  Serial.print(" | Sensors:[");
  for (int i = 0; i < SensorCount; i++) {
    Serial.print(sensorValues[i]);
    if (i < SensorCount - 1) Serial.print(",");
  }
  Serial.println("]");
}

// ========== SETUP ==========

void setup() {
  pinMode(3, OUTPUT); // left motor
  pinMode(2, OUTPUT); // left motor
  Serial.begin(115200);

  // Setup QTRX line sensors
  // IMPORTANT: QTRX sensors use RC (digital) mode, not analog
  qtr.setTypeRC();

  // QTRX Sensor Array pin configuration for TI-RSLK / A-Star
  // Sensors 0-7 connect to these pins:
  // NOTE: REMOVE THE BUZZER JUMPER ON THE A-STAR!
  qtr.setSensorPins((const uint8_t[]){7, 18, 23, 20, 21, 22, 8, 7}, SensorCount);

  // QTRX has TWO emitter control pins
  // CTRL EVEN (pin 5) controls sensors 0, 2, 4, 6
  // CTRL ODD (pin 4) controls sensors 1, 3, 5, 7
  qtr.setEmitterPins(5, 4); // (CTRL_EVEN, CTRL_ODD)

  pinMode(LED_BUILTIN, OUTPUT);

  // Wait 5 seconds before calibration
  Serial.println("=== LINE FOLLOWING TEST (QTRX) ===");
  Serial.println("IMPORTANT: Remove BUZZER jumper!");
  Serial.println("Starting in 5 seconds...");
  digitalWrite(LED_BUILTIN, HIGH);
  delay(5000);
  digitalWrite(LED_BUILTIN, LOW);

  // Calibrate sensors
  calibrateSensors();

  Serial.println("=== READY TO FOLLOW LINE ===");
  Serial.println("Place robot on line and it will start following.");
  Serial.println("<Arduino is ready>");
  delay(2000);
}

// ========== MAIN LOOP ==========

void loop() {
  if (!lineFollowingComplete) {
    // Execute line following
    executeLineFollowing();

    // Command motors
    commandMotors();

    // Publish sensor data every loop
    publishSensorData();

    delay(20);  // Small delay for stability
  } else {
    // Line following complete - stay stopped
    stopMotors();

    // Print completion message periodically
    static unsigned long lastPrintTime = 0;
    if (millis() - lastPrintTime > 2000) {
      Serial.println(">>> STOPPED AT SECOND CROSS - Test Complete!");
      lastPrintTime = millis();
    }
  }
}


