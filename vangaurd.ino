#include <AccelStepper.h>
#include <Servo.h>
#include <math.h>

// =============================
// PIN DEFINITIONS
// =============================

// ---- Joystick pins (analog) ----
const int VRx = A2;  // Joystick X
const int VRy = A3;  // Joystick Y

// ---- L298 pins (DC motors) ----
// Motor A
const int IN1 = 22;
const int IN2 = 23;
const int ENA =  2;   // PWM capable on Mega

// Motor B
const int IN3 = 24;
const int IN4 = 25;
const int ENB =  3;   // PWM capable on Mega

// ---- Buttons ----
const int leftButton  = A0;
const int rightButton = A1;

// ---- Ultrasonic Sensor ----
int trigPin = 36;
int echoPin = 37;

// ---- Servos ----
// servo1 = arm tilt     ; 
// servo2 = end effector ;
Servo servo1, servo2;

// ---- Steppers ----
// stepper1 = joint 1; stepper 2 = joint 2; stepper 3 = base
AccelStepper stepper1(AccelStepper::DRIVER, 30, 31);  // STEP=30, DIR=31
AccelStepper stepper2(AccelStepper::DRIVER, 32, 33);  // STEP=32, DIR=33
AccelStepper stepper3(AccelStepper::DRIVER, 34, 35);  // STEP=34, DIR=35

// =============================
// GLOBALS & CONSTANTS
// =============================

// Link lengths
float L[3] = {0.25, 0.15, 0.15};
float ultrasonic_offset = 0.115;

// Control state
bool control_done = false;

// Stepper conversion
const float steps_per_rev = 200.0                        ;// adjust to your motor
const float degrees_per_step = 360.0 / steps_per_rev     ;
const float radians_per_step = PI / (steps_per_rev / 2.0);

// ---- Drive override from serial ----
bool driveOverrideActive = false;
int overrideLeftSpeed  = 0      ;   // -255..255
int overrideRightSpeed = 0      ;   // -255..255

// ---- Serial hex state machine ----
char pendingCommand = 0     ; // stores first hex char (command)
bool waitingForParam = false; // true = next hex char is parameter

// =============================
// FORWARD DECLARATIONS
// =============================

// DC motors (L298)
void runMotor(int pin1, int pin2, int enablePin, int speedVal);
void joystickDrive()                                          ;

// Arm control
float ultrasonic()                                                            ;
void fkine(float theta[3], float end_effector[2], float joint_positions[4][2]);
void jacobian_xy_analytic(float theta[3], float J[2][3])                      ;
void target_position(float target[2])                                         ;
void get_final_angles(float target[2], float q_out[3])                        ;
void arm_position()                                                           ;
void direction()                                                              ;
void move_obstacle_left()                                                     ;
void move_obstacle_right()                                                    ;
void end_effector_open()                                                      ;
void end_effector_close()                                                     ;

// Serial hex command handling
void handleSerialProtocol()              ;
bool isHexChar(char c)                   ;
int  hexCharToNibble(char c)             ;
void dispatchCommand(char cmd, int level);

// Example command handlers (A/B real, others placeholders)
void cmd_A(int level);
void cmd_B(int level);
void cmd_C(int level);
void cmd_D(int level);
void cmd_E(int level);
void cmd_F(int level);
void cmd_0(int level);
void cmd_1(int level);
void cmd_2(int level);
void cmd_3(int level);
void cmd_4(int level);
void cmd_5(int level);
void cmd_6(int level);
void cmd_7(int level);
void cmd_8(int level);
void cmd_9(int level);

// =============================
// SETUP
// =============================

void setup() {
  Serial.begin(9600);

  // ---- L298 outputs ----
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(ENA, OUTPUT);

  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENB, OUTPUT);

  // ---- Ultrasonic Sensor setup ----
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT) ;

  // ---- Buttons ----
  pinMode(leftButton, INPUT_PULLUP) ;
  pinMode(rightButton, INPUT_PULLUP);

  // ---- Servos ----
  servo1.attach(9) ;   // adjust if needed
  servo2.attach(10);  // adjust if needed
  servo1.write(0)  ;
  servo2.write(90) ;

  // ---- Steppers ----
  stepper1.setMaxSpeed(1000);
  stepper2.setMaxSpeed(1000);
  stepper3.setMaxSpeed(1000);

  stepper1.setAcceleration(500);
  stepper2.setAcceleration(500);
  stepper3.setAcceleration(500);
}

// =============================
// MAIN LOOP
// =============================

void loop() {
  // 1) Serial protocol is fully non-blocking
  handleSerialProtocol();

  // 2) Drive base (either joystick or serial override)
  joystickDrive();

  // 3) Arm control (still has some blocking pieces in your original logic)
  arm_position();

  // 4) Left/right obstacle motion via buttons
  direction();

  // Small delay to ease CPU load
  delay(5);
}

// =============================
// JOYSTICK + L298 (Script 1 with override)
// =============================

void joystickDrive() {
  int leftMotorSpeed, rightMotorSpeed;

  if (!driveOverrideActive) {
    int xVal = analogRead(VRx); // 0 - 1023
    int yVal = analogRead(VRy);

    // Convert to range -255 to +255
    int speedY = map(yVal, 0, 1023, 255, -255);
    int turnX  = map(xVal, 0, 1023, -255, 255);

    // Determine individual motor speeds
    leftMotorSpeed  = speedY + turnX;
    rightMotorSpeed = speedY - turnX;

    // Constrain output to motor range
    leftMotorSpeed  = constrain(leftMotorSpeed, -255, 255) ;
    rightMotorSpeed = constrain(rightMotorSpeed, -255, 255);
  } 
  else {
    // Serial override: ignore joystick, use commanded speeds
    leftMotorSpeed  = overrideLeftSpeed ;
    rightMotorSpeed = overrideRightSpeed;
  }

  // Drive motors
  runMotor(IN1, IN2, ENA, leftMotorSpeed) ;
  runMotor(IN3, IN4, ENB, rightMotorSpeed);
}

void runMotor(int pin1, int pin2, int enablePin, int speedVal) {
       if (speedVal > 0) {
    digitalWrite(pin1, HIGH)        ;
    digitalWrite(pin2, LOW)         ;
    analogWrite(enablePin, speedVal);
  } 
  else if (speedVal < 0) {
    digitalWrite(pin1, LOW)          ;
    digitalWrite(pin2, HIGH)         ;
    analogWrite(enablePin, -speedVal);
  } 
  else {
    digitalWrite(pin1, LOW)  ;
    digitalWrite(pin2, LOW)  ;
    analogWrite(enablePin, 0);
  }
}

// =============================
// ULTRASONIC + ARM (Script 2)
// =============================

float ultrasonic() {
  float distance_m ;
  float duration_us;

  // generate 10-microsecond pulse to TRIG pin
  digitalWrite(trigPin, LOW) ;
  delayMicroseconds(2)       ;
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10)      ;
  digitalWrite(trigPin, LOW) ;

  // measure duration of pulse from ECHO pin
  duration_us = pulseIn(echoPin, HIGH);

  // calculate the distance (approx)
  distance_m = 0.017 * duration_us / 100 + ultrasonic_offset;

  Serial.print("distance: ");
  Serial.print(distance_m)  ;
  Serial.println(" m")      ;

  return distance_m;
}

// Forward kinematics
void fkine(float theta[3], float end_effector[2], float joint_positions[4][2]) {
  float th1 = theta[0], th2 = theta[1], th3 = theta[2];
  float l1 = L[0], l2 = L[1], l3 = L[2]               ;

  joint_positions[0][0] = 0.0                                              ;  
  joint_positions[0][1] = 0.0                                              ;
  joint_positions[1][0] = l1 * cos(th1)                                    ; 
  joint_positions[1][1] = l1 * sin(th1)                                    ;
  joint_positions[2][0] = joint_positions[1][0] + l2 * cos(th1 + th2)      ;
  joint_positions[2][1] = joint_positions[1][1] + l2 * sin(th1 + th2)      ;
  joint_positions[3][0] = joint_positions[2][0] + l3 * cos(th1 + th2 + th3);
  joint_positions[3][1] = joint_positions[2][1] + l3 * sin(th1 + th2 + th3);

  end_effector[0] = joint_positions[3][0];
  end_effector[1] = joint_positions[3][1];
}

// Jacobian
void jacobian_xy_analytic(float theta[3], float J[2][3]) {
  float th1 = theta[0], th2 = theta[1], th3 = theta[2];
  float l1  = L[0], l2 = L[1], l3 = L[2]              ;

  float c1 = cos(th1), s1 = sin(th1)                        ;
  float c2 = cos(th1 + th2), s2 = sin(th1 + th2)            ;
  float c3 = cos(th1 + th2 + th3), s3 = sin(th1 + th2 + th3);

  J[0][0] = -l1 * s1 - l2 * s2 - l3 * s3;
  J[0][1] = -l2 * s2 - l3 * s3          ;
  J[0][2] = -l3 * s3                    ;
  J[1][0] = l1 * c1 + l2 * c2 + l3 * c3 ;
  J[1][1] = l2 * c2 + l3 * c3           ;
  J[1][2] = l3 * c3                     ;
}

void target_position(float target[2]) {
  float distance_m = ultrasonic();
  target[0]        = distance_m  ;
  target[1]        = 0.15        ;
}

// Resolved-rate control loop
void get_final_angles(float target[2], float q_out[3]) {
  float q[3] = {0.0, 0.0, 0.0};
  float K    = 2.0            ;
  int step   = 500            ;
  float dt   = 0.03           ;
  float position[2]           ;
  float joint_positions[4][2] ;

  fkine(q, position, joint_positions);

  for (int i = 0; i < step; i++) {
    float error[2]  = {target[0] - position[0], target[1] - position[1]};
    float error_mag = sqrt(error[0]*error[0] + error[1]*error[1])       ;
    if (error_mag < 0.00001) {
      Serial.println("Target reached.");
      break                            ;
    }

    float v[2] = {K * error[0], K * error[1]};
    float J[2][3]                            ;
    jacobian_xy_analytic(q, J)               ;
    float dq[3]                              ;

    for (int j = 0; j < 3; j++) {
      dq[j] = J[0][j] * v[0] + J[1][j] * v[1];
      q[j] += dq[j] * dt                     ;
    }

    fkine(q, position, joint_positions);
  }

  for (int i = 0; i < 3; i++) {q_out[i] = q[i];}
}

void arm_position() {
  if (!control_done) {
    float final_q[3]                 ;
    float target[2]                  ;
    target_position(target)          ;
    Serial.print("target: ")         ;
    Serial.println(target[0])        ;
    get_final_angles(target, final_q);

    // Convert radians to steps
    long steps1 = final_q[0] / radians_per_step;
    long steps2 = final_q[1] / radians_per_step;

    Serial.print("Final joint angles (deg): ")             ;
    Serial.print(steps1 * degrees_per_step)                ;
    Serial.print(" ")                                      ;
    Serial.print(steps2 * degrees_per_step)                ;
    Serial.print(" ")                                      ;
    Serial.print(90 - (final_q[0] + final_q[1]) * 180 / PI);
    Serial.println()                                       ;

    // Move steppers
    stepper1.moveTo(steps1)                                ;
    stepper2.moveTo(steps2)                                ;
    servo1.write(90 - (final_q[0] + final_q[1]) * 180 / PI);

    while (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning()) {
      stepper1.run();
      stepper2.run();
      stepper3.run();
    }

    Serial.print("Stepper target steps: ") ;
    Serial.print(steps1); Serial.print(" ");
    Serial.print(steps2); Serial.print(" ");

    control_done = true;
  }

  while (stepper1.isRunning() || stepper2.isRunning() || stepper3.isRunning()) {
    stepper1.run();
    stepper2.run();
    stepper3.run();
  }
}

void direction() {
  if (digitalRead(rightButton) == LOW) {
    Serial.println("right button: 0");
    move_obstacle_right()            ;
  }
  if (digitalRead(leftButton) == LOW) {
    Serial.println("left button: 0");
    move_obstacle_left()            ;
  }
}

void move_obstacle_left() {
  long steps3 = - PI / (2 * radians_per_step) ;
  end_effector_open()                         ;
  delay(500)                                  ;
  stepper3.moveTo(steps3)                     ;
  while (stepper3.isRunning()) {stepper3.run();}
  delay(100)                                  ;
  end_effector_close()                        ;
  delay(1000)                                 ;
  stepper3.moveTo(0)                          ;
  while (stepper3.isRunning()) {stepper3.run();}
}

void move_obstacle_right() {
  long steps3 = PI / (2 * radians_per_step)   ;
  end_effector_open()                         ;
  delay(500)                                  ;
  stepper3.moveTo(steps3)                     ;
  while (stepper3.isRunning()) {stepper3.run();}
  delay(100)                                  ;
  end_effector_open()                         ;
  end_effector_close()                        ;
  delay(1000)                                 ;
  stepper3.moveTo(0)                          ;
  while (stepper3.isRunning()) {stepper3.run();}
}

void end_effector_open()  {servo2.write(30);}
void end_effector_close() {servo2.write(90);}

// =============================
// SERIAL HEX STATE MACHINE
// =============================
//
// Protocol:
//   First non-whitespace hex char = command
//   Second hex char               = level (0-15)
// Example:
//   "A9" => command 'A', level 9   (≈90%)
//   "B3" => command 'B', level 3   (≈30%)
//   "00" => command '0', level 0   (we use this to clear override)
//

void handleSerialProtocol() {
  while (Serial.available() > 0) {
    char c = Serial.read();

    // Skip whitespace
    if (c == '\n' || c == '\r' || c == ' ' || c == '\t') {
      continue;
    }

    if (!waitingForParam) {
      // Expecting a command
      if (isHexChar(c)) {
        pendingCommand  = c   ;
        waitingForParam = true;
        // Serial.print("Got command: "); Serial.println(pendingCommand);
      } else {
        Serial.print("Ignored non-hex command char: ");
        Serial.println(c)                             ;
      }
    } else {
      // Expecting a parameter
      if (isHexChar(c)) {
        int level = hexCharToNibble(c);  // 0..15
        // Serial.print("Got param: "); Serial.println(level);
        dispatchCommand(pendingCommand, level);
      } else {
        Serial.print("Ignored non-hex param char: ");
        Serial.println(c)                           ;
      }
      // Reset state after second char, regardless
      waitingForParam = false;
      pendingCommand  = 0    ;
    }
  }
}

bool isHexChar(char c) {
  return (c >= '0' && c <= '9') ||
         (c >= 'A' && c <= 'F') ||
         (c >= 'a' && c <= 'f');
}

int hexCharToNibble(char c) {
  if (c >= '0' && c <= '9') return c - '0'       ;
  if (c >= 'A' && c <= 'F') return 10 + (c - 'A');
  if (c >= 'a' && c <= 'f') return 10 + (c - 'a');
  return 0                                       ;
}

void dispatchCommand(char cmd, int level) {
  // Normalize cmd to uppercase
  if (cmd >= 'a' && cmd <= 'f') {cmd = cmd - 'a' + 'A';}

  Serial.print("Dispatch cmd=");
  Serial.print(cmd)            ;
  Serial.print(" level=")      ;
  Serial.println(level)        ;

  switch (cmd) {
    case 'A': cmd_A(level); break;
    case 'B': cmd_B(level); break;
    case 'C': cmd_C(level); break;
    case 'D': cmd_D(level); break;
    case 'E': cmd_E(level); break;
    case 'F': cmd_F(level); break;
    case '0': cmd_0(level); break;
    case '1': cmd_1(level); break;
    case '2': cmd_2(level); break;
    case '3': cmd_3(level); break;
    case '4': cmd_4(level); break;
    case '5': cmd_5(level); break;
    case '6': cmd_6(level); break;
    case '7': cmd_7(level); break;
    case '8': cmd_8(level); break;
    case '9': cmd_9(level); break;
    default:
      Serial.print("Unknown command: ");
      Serial.println(cmd)              ;
      break                            ;
  }
}

// =============================
// COMMAND IMPLEMENTATIONS
// =============================
//
// Example semantics:
//   A<hex> => drive forward with that speed
//   B<hex> => drive backward with that speed
//   0<hex> => clear override (back to joystick)
//
// Level mapping:
//   level 0..9 => ~0..90%
//   level A..F => clamp to 100%
//

int levelToPercent(int level) {
  int percent = level * 10        ;    // 0 ->0%, 9->90%, A-F ->100+ but we’ll clamp
  if (percent > 100) percent = 100;
  if (percent < 0)   percent = 0  ;
  return percent                  ;
}

int percentToPWM(int percent) {return map(percent, 0, 100, 0, 255);}

void cmd_A(int level) {
  int pct = levelToPercent(level);
  int pwm = percentToPWM(pct)    ;

  driveOverrideActive = true;
  overrideLeftSpeed  = pwm  ;
  overrideRightSpeed = pwm  ;

  Serial.print("cmd_A: forward "); Serial.print(pct); Serial.println("%");
}

void cmd_B(int level) {
  int pct = levelToPercent(level);
  int pwm = percentToPWM(pct)    ;

  driveOverrideActive = true;
  overrideLeftSpeed  = -pwm ;
  overrideRightSpeed = -pwm ;

  Serial.print("cmd_B: backward "); 
  Serial.print(pct)               ;  
  Serial.println("%")             ;
}

// Placeholders – add your own logic per command if needed
void cmd_C(int level) {Serial.print("cmd_C (placeholder), level="); Serial.println(level);}
void cmd_D(int level) {Serial.print("cmd_D (placeholder), level="); Serial.println(level);}
void cmd_E(int level) {Serial.print("cmd_E (placeholder), level="); Serial.println(level);}
void cmd_F(int level) {Serial.print("cmd_F (placeholder), level="); Serial.println(level);}

// '0' – here we use as "clear drive override", but you can change
void cmd_0(int level) {
  driveOverrideActive = false                                ;
  overrideLeftSpeed   = 0                                    ;
  overrideRightSpeed  = 0                                    ;
  Serial.println("cmd_0: override cleared, back to joystick");
}

// Other digits just print by default
void cmd_1(int level) { Serial.print("cmd_1 (placeholder), level="); Serial.println(level); }
void cmd_2(int level) { Serial.print("cmd_2 (placeholder), level="); Serial.println(level); }
void cmd_3(int level) { Serial.print("cmd_3 (placeholder), level="); Serial.println(level); }
void cmd_4(int level) { Serial.print("cmd_4 (placeholder), level="); Serial.println(level); }
void cmd_5(int level) { Serial.print("cmd_5 (placeholder), level="); Serial.println(level); }
void cmd_6(int level) { Serial.print("cmd_6 (placeholder), level="); Serial.println(level); }
void cmd_7(int level) { Serial.print("cmd_7 (placeholder), level="); Serial.println(level); }
void cmd_8(int level) { Serial.print("cmd_8 (placeholder), level="); Serial.println(level); }
void cmd_9(int level) { Serial.print("cmd_9 (placeholder), level="); Serial.println(level); }
