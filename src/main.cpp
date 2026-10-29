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
const int forwardSpeed = 150; // Speed when driving straight
const int turnSpeed = 150;    // Speed when turning

// Speed aliases
const int MOTOR_SPEED_FORWARD = forwardSpeed;
const int MOTOR_SPEED_TURN = turnSpeed;

// ----------------- SENSOR THRESHOLDS & GAP -----------------
const int THRESHOLD_WHITE = 600;                   // 0 to 600 is White
const int GAP = 50;                                // 50-point deadband gap
const int THRESHOLD_BLACK = THRESHOLD_WHITE + GAP; // 650 and above is Black

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
 * @brief Pivot-turn robot left by driving right motor and stopping left motor.
 * @param speed PWM speed duty cycle for the active motor (0 - 255).
 */
void turnLeft(int speed = turnSpeed) {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, 0);
  analogWrite(ENB, speed);
}

/**
 * @brief Pivot-turn robot right by driving left motor and stopping right motor.
 * @param speed PWM speed duty cycle for the active motor (0 - 255).
 */
void turnRight(int speed = turnSpeed) {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);

  analogWrite(ENA, speed);
  analogWrite(ENB, 0);
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
 * @details ATmega328P shares a single ADC via multiplexer. A dummy read followed
 *          by a 20us settling time prevents sample-and-hold capacitor ghosting
 *          when switching between adjacent analog channels.
 * @param pin The analog pin to read (e.g. A3, A4, A5).
 * @return Settled 10-bit analog conversion value (0 - 1023).
 */
int readSensor(int pin) {
  analogRead(pin);        // Dummy read to switch ADC multiplexer channel
  delayMicroseconds(20);  // Allow ADC sample-and-hold capacitor to settle
  return analogRead(pin); // Accurate reading
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
  Serial.println(F(" Analog Line Follower Started"));
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

  // ── Line Following Logic ───────────────────────────────────

  // Stop when all 3 IR sensors detect Black (intersection / stop line)
  if (leftBlack && centerBlack && rightBlack) {
    stopMotors();
  }
  // Left sensor on line -> Turn Left
  else if (leftBlack && !rightBlack) {
    turnLeft(MOTOR_SPEED_TURN);
  }
  // Right sensor on line -> Turn Right
  else if (rightBlack && !leftBlack) {
    turnRight(MOTOR_SPEED_TURN);
  }
  // Line centered (Center on black) or both off line -> Move Forward
  else {
    moveForward(MOTOR_SPEED_FORWARD);
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
    Serial.println(sensorRight);
  }
}

