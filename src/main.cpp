/**
 * @file main.cpp
 * @brief 3-Sensor Analog Line Follower Robot Firmware
 * @details Autonomous line follower using an Arduino Uno, L298N dual H-bridge
 *          motor driver, and a 3-channel analog IR sensor array. Implements
 *          ADC settling delay (crosstalk elimination) and hysteresis deadband
 *          to prevent oscillation.
 *
 * Microcontroller: Arduino Uno (ATmega328P)
 * Framework: Arduino / PlatformIO
 */

#include <Arduino.h>

// ----------------- PIN DEFINITIONS -----------------
#define ir_left A3   // Analog pin for Left IR sensor
#define ir_center A4 // Analog pin for Center IR sensor
#define ir_right A5  // Analog pin for Right IR sensor

// Pin aliases
#define IR_SENSOR_LEFT ir_left
#define IR_SENSOR_CENTER ir_center
#define IR_SENSOR_RIGHT ir_right

// L298N Motor Driver Pins
#define ENA 9 // Left motor speed (PWM)
#define ENB 3 // Right motor speed (PWM)
#define IN1 8 // Left motor direction 1
#define IN2 7 // Left motor direction 2
#define IN3 5 // Right motor direction 1
#define IN4 4 // Right motor direction 2

// ----------------- SPEED SETTINGS (0 - 255) -----------------
const int forwardSpeed = 255;  // Speed when driving straight
const int turnSpeed = 255;     // Speed when turning
const int backwardSpeed = 150; // Speed when reversing
const unsigned long backwardDuration =
    500; // Duration to reverse at destination (ms)
const unsigned long blindTurnDuration =
    1000; // Duration to turn without detecting line (1s)

// Speed aliases
const int MOTOR_SPEED_FORWARD = forwardSpeed;
const int MOTOR_SPEED_TURN = turnSpeed;
const int MOTOR_SPEED_BACKWARD = backwardSpeed;

// ----------------- SENSOR THRESHOLDS & GAP -----------------
const int THRESHOLD_WHITE = 600;                   // 0 to 600 is White
const int GAP = 50;                                // 50-point deadband gap
const int THRESHOLD_BLACK = THRESHOLD_WHITE + GAP; // 650 and above is Black

// ----------------- MISSION STATES -----------------
enum MissionState {
  STATE_TO_GOAL,         // Phase 1: Follow line towards the destination
  STATE_REVERSE_AT_GOAL, // Phase 2: Destination reached: move back for 3
                         // seconds
  STATE_TURN_AROUND,     // Phase 3: Turn right to locate and lock onto line
  STATE_TO_START,        // Phase 4: Follow line back towards starting point
  STATE_COMPLETED        // Phase 5: Reached starting point: stop permanently
};

MissionState missionState = STATE_TO_GOAL;

// ----------------- MOTOR FUNCTIONS -----------------

/**
 * @brief Drive both motors forward at the given PWM speed.
 * @param speed PWM speed duty cycle (0 - 255), defaults to forwardSpeed.
 */
void forward(int speed = forwardSpeed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

/**
 * @brief Semantic alias for forward().
 * @param speed PWM speed duty cycle (0 - 255).
 */
void moveForward(int speed = forwardSpeed) { forward(speed); }

/**
 * @brief Drive both motors backward at the given PWM speed.
 * @param speed PWM speed duty cycle (0 - 255), defaults to backwardSpeed.
 */
void backward(int speed = backwardSpeed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);

  analogWrite(ENA, speed);
  analogWrite(ENB, speed);
}

/**
 * @brief Semantic alias for backward().
 * @param speed PWM speed duty cycle (0 - 255).
 */
void moveBackward(int speed = backwardSpeed) { backward(speed); }

/**
 * @brief Spin-turn robot left using both motors (left backward, right forward).
 * @param speed PWM speed duty cycle for both motors (0 - 255).
 */
void turnLeft(int speed = turnSpeed) {
  // Left motor reverse
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);
  analogWrite(ENA, speed);

  // Right motor forward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);
  analogWrite(ENB, speed);
}

/**
 * @brief Spin-turn robot right using both motors (left forward, right
 * backward).
 * @param speed PWM speed duty cycle for both motors (0 - 255).
 */
void turnRight(int speed = turnSpeed) {
  // Left motor forward
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  analogWrite(ENA, speed);

  // Right motor reverse
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);
  analogWrite(ENB, speed);
}

/**
 * @brief Immediately stop both motors and disable PWM drive signals.
 */
void stopMotors() {
  analogWrite(ENA, 0);
  analogWrite(ENB, 0);

  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

// ----------------- SENSOR READ FUNCTION -----------------
/**
 * @brief Read analog pin with ADC multiplexer settling delay.
 * @details ATmega328P shares a single ADC via multiplexer. A dummy read
 * followed by a 20us settling time prevents sample-and-hold capacitor ghosting
 *          when switching between adjacent analog channels.
 * @param pin The analog pin to read (e.g. A3, A4, A5).
 * @return Settled 10-bit analog conversion value (0 - 1023).
 */
int readSensor(int pin) {
  analogRead(pin);        // Dummy read to switch ADC multiplexer channel
  delayMicroseconds(20);  // Allow ADC sample-and-hold capacitor to settle
  return analogRead(pin); // Accurate reading
}

// ----------------- MISSION NAVIGATION HELPERS -----------------
/**
 * @brief Turn robot right in place using both motors until it finds the line
 * again.
 * @details 1. Starts turning right with both motors.
 *          2. Blind-turns for 1 second (blindTurnDuration) ignoring all
 * sensors.
 *          3. After 1s, monitors sensors until the black line is detected.
 *          4. Brakes to settle momentum.
 */
void turnRightToFindLine() {
  Serial.println(F("[MISSION] Turning right (ignoring line for 1s)..."));

  // 1. Begin in-place right turn using both motors
  turnRight(170);

  // 2. Ignore line detection for the first 1 second (blind turn)
  delay(blindTurnDuration); // 1000 ms

  Serial.println(F("[MISSION] 1s elapsed. Seeking line now..."));

  // 3. Keep turning right until center sensor re-acquires the black line
  unsigned long tStart = millis();
  while (readSensor(IR_SENSOR_CENTER) < THRESHOLD_BLACK) {
    // If left sensor detects black first, catch it before overshooting
    if (readSensor(IR_SENSOR_LEFT) >= THRESHOLD_BLACK) {
      break;
    }
    if (millis() - tStart > 4000) {
      break; // Safety timeout (4.0 sec)
    }
    delay(5);
  }

  // 4. Brief brake to stabilize robot alignment
  stopMotors();
  delay(150);
}

// ----------------- SETUP -----------------
void setup() {
  // Set IR sensors as INPUT
  pinMode(ir_left, INPUT);
  pinMode(ir_center, INPUT);
  pinMode(ir_right, INPUT);

  // Set Motor control pins as OUTPUT
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  Serial.begin(9600);
  stopMotors();

  Serial.println(F("========================================"));
  Serial.println(F(" Analog Line Follower Mission"));
  Serial.println(F(" Forward -> Move Back 3s -> Turn Right -> Return"));
  Serial.println(F(" White <= 600 | Gap: 50 | Black >= 650"));
  Serial.println(F("========================================"));

  delay(1000); // 1 second startup delay
}

// ----------------- MAIN LOOP -----------------
void loop() {
  // Read analog values from IR sensors (with ADC channel settling)
  int sensorLeft = readSensor(IR_SENSOR_LEFT);
  int sensorCenter = readSensor(IR_SENSOR_CENTER);
  int sensorRight = readSensor(IR_SENSOR_RIGHT);

  // Hysteresis with 50-point gap (prevents noise and jitter between 600 and
  // 650) Becomes Black when >= 650, becomes White when <= 600
  static bool leftBlack = false;
  static bool centerBlack = false;
  static bool rightBlack = false;

  if (sensorLeft >= THRESHOLD_BLACK)
    leftBlack = true;
  else if (sensorLeft <= THRESHOLD_WHITE)
    leftBlack = false;

  if (sensorCenter >= THRESHOLD_BLACK)
    centerBlack = true;
  else if (sensorCenter <= THRESHOLD_WHITE)
    centerBlack = false;

  if (sensorRight >= THRESHOLD_BLACK)
    rightBlack = true;
  else if (sensorRight <= THRESHOLD_WHITE)
    rightBlack = false;

  // Track mission progress flags
  static bool departedStart = false;
  static unsigned long returnStartTime = 0;

  // ── Mission State Machine ───────────────────────────────────
  switch (missionState) {
  // 1. Move forward along line towards the goal
  case STATE_TO_GOAL: {
    // Flag that robot left initial start position (if placed on all-black line)
    if (!(leftBlack && centerBlack && rightBlack)) {
      departedStart = true;
    }

    // Goal position reached: all 3 IR sensors detect Black
    if (departedStart && leftBlack && centerBlack && rightBlack) {
      Serial.println(F("[MISSION] Goal reached! All IR HIGH."));
      stopMotors();
      missionState = STATE_REVERSE_AT_GOAL;
    }
    // Follow the line
    else if (leftBlack && !rightBlack) {
      turnLeft(MOTOR_SPEED_TURN);
    } else if (rightBlack && !leftBlack) {
      turnRight(MOTOR_SPEED_TURN);
    } else {
      moveForward(MOTOR_SPEED_FORWARD);
    }
    break;
  }

  // 2. Reached destination: move back for 3 seconds, then start turnaround
  case STATE_REVERSE_AT_GOAL: {
    stopMotors();
    delay(200); // Brief pause to protect H-bridge before reversing

    Serial.println(
        F("[MISSION] Destination reached! Moving back for 3 seconds..."));
    moveBackward(MOTOR_SPEED_BACKWARD);
    delay(backwardDuration); // Reverse for 3 seconds

    stopMotors();
    delay(200); // Settle after reversing
    Serial.println(
        F("[MISSION] Reverse complete. Starting turnaround process..."));
    missionState = STATE_TURN_AROUND;
    break;
  }

  // 3. Turn right and find the line
  case STATE_TURN_AROUND: {
    turnRightToFindLine();
    Serial.println(F("[MISSION] Line locked! Returning to start..."));
    // Reset sensor hysteresis states for return trip
    leftBlack = false;
    centerBlack = true;
    rightBlack = false;
    returnStartTime = millis();
    missionState = STATE_TO_START;
    break;
  }

  // 4. Follow line back towards starting point
  case STATE_TO_START: {
    // Allow 800ms of return travel before detecting starting line stop
    // condition
    bool canDetectStart = (millis() - returnStartTime > 800);

    // Starting point reached: all 3 IR sensors detect Black
    if (canDetectStart && leftBlack && centerBlack && rightBlack) {
      Serial.println(F("[MISSION] Starting point reached! Mission completed."));
      stopMotors();
      missionState = STATE_COMPLETED;
    }
    // Follow the line
    else if (leftBlack && !rightBlack) {
      turnLeft(MOTOR_SPEED_TURN);
    } else if (rightBlack && !leftBlack) {
      turnRight(MOTOR_SPEED_TURN);
    } else {
      moveForward(MOTOR_SPEED_FORWARD);
    }
    break;
  }

  // 5. Mission finished: remain stopped
  case STATE_COMPLETED: {
    stopMotors();
    break;
  }
  }

  // Debugging output over Serial (throttled every 100ms to avoid blocking loop)
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint >= 100) {
    lastPrint = millis();
    Serial.print(F("L:"));
    Serial.print(sensorLeft);
    Serial.print(F(" C:"));
    Serial.print(sensorCenter);
    Serial.print(F(" R:"));
    Serial.print(sensorRight);
    Serial.print(F(" | Mission:"));
    switch (missionState) {
    case STATE_TO_GOAL:
      Serial.println(F("TO_GOAL"));
      break;
    case STATE_REVERSE_AT_GOAL:
      Serial.println(F("MOVE_BACK_3S"));
      break;
    case STATE_TURN_AROUND:
      Serial.println(F("TURN_AROUND"));
      break;
    case STATE_TO_START:
      Serial.println(F("TO_START"));
      break;
    case STATE_COMPLETED:
      Serial.println(F("COMPLETED"));
      break;
    }
  }
}
