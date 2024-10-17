#include <util/atomic.h>

// Pins for motor 1 (left)
#define ENCA1 2
#define ENCB1 7
#define PWM1 13
#define IN1_1 8
#define IN2_1 9

// Pins for motor 2 (right)
#define ENCA2 3
#define ENCB2 6
#define PWM2 12
#define IN1_2 11
#define IN2_2 10

// Globals for motor 1
long prevT1 = 0;
int posPrev1 = 0;
volatile int pos_i1 = 0;
float v1Filt = 0;
float v1Prev = 0;
float eintegral1 = 0;

// Globals for motor 2
long prevT2 = 0;
int posPrev2 = 0;
volatile int pos_i2 = 0;
float v2Filt = 0;
float v2Prev = 0;
float eintegral2 = 0;

// Target variables
float vt1 = 0;
float vt2 = 0;

// Robot parameters
const float wheel_radius = 0.04;  // Wheel radius in meters
const float wheel_track = 0.35;   // Distance between wheels in meters

// Integral limits to prevent windup
const float max_integral = 100.0;

void setup() {
  Serial.begin(115200);  // Start serial communication
  clearSerialBuffer();    // Ensure serial buffer is clean

  // Motor 1 setup
  pinMode(ENCA1, INPUT);
  pinMode(ENCB1, INPUT);
  pinMode(PWM1, OUTPUT);
  pinMode(IN1_1, OUTPUT);
  pinMode(IN2_1, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA1), readEncoder1, RISING);

  // Motor 2 setup
  pinMode(ENCA2, INPUT);
  pinMode(ENCB2, INPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(IN1_2, OUTPUT);
  pinMode(IN2_2, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA2), readEncoder2, RISING);

  // Notify ROS that the Arduino is ready
  Serial.println("READY");
}

void loop() {
  if (Serial.available()) {
    String inputString = Serial.readStringUntil('\n');
    parseInput(inputString);
  }

  // Calculate motor velocities
  float linear_x, angular_z;
  calculateVelocities(linear_x, angular_z);

  // Send velocities to ROS 2
  sendVelocityFeedback(linear_x, angular_z);

  // Set motor speeds
  controlMotors();

  delay(1);
}

void parseInput(const String &input) {
  int separatorIndex = input.indexOf(',');
  if (separatorIndex != -1) {
    vt1 = input.substring(0, separatorIndex).toFloat();
    vt2 = input.substring(separatorIndex + 1).toFloat();
  } else {
    // Handle malformed input gracefully
    Serial.println("ERR: Malformed input");
  }
}

void calculateVelocities(float &linear_x, float &angular_z) {
  int pos1, pos2;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    pos1 = pos_i1;
    pos2 = pos_i2;
  }

  long currT1 = micros();
  float deltaT1 = (currT1 - prevT1) / 1.0e6;
  float v1 = ((pos1 - posPrev1) / deltaT1) / 2400.0 * 60.0;
  posPrev1 = pos1;
  prevT1 = currT1;

  long currT2 = micros();
  float deltaT2 = (currT2 - prevT2) / 1.0e6;
  float v2 = ((pos2 - posPrev2) / deltaT2) / 2400.0 * 60.0;
  posPrev2 = pos2;
  prevT2 = currT2;

  v1Filt = 0.854 * v1Filt + 0.0728 * v1 + 0.0728 * v1Prev;
  v1Prev = v1;
  v2Filt = 0.854 * v2Filt + 0.0728 * v2 + 0.0728 * v2Prev;
  v2Prev = v2;

  float v1_linear = (v1Filt * 2 * 3.14159 / 60.0) * wheel_radius;
  float v2_linear = (v2Filt * 2 * 3.14159 / 60.0) * wheel_radius;

  linear_x = (v1_linear + v2_linear) / 2;
  angular_z = (v2_linear - v1_linear) / wheel_track;
}

void sendVelocityFeedback(float linear_x, float angular_z) {
  String feedback = "<" + String(linear_x, 2) + "," + String(angular_z, 2) + ">\n";
  Serial.print(feedback);
}

void controlMotors() {
  float u1 = constrain(1.5 * (vt1 - v1Filt) + 15 * eintegral1, -255, 255);
  eintegral1 = constrain(eintegral1 + (vt1 - v1Filt) * ((micros() - prevT1) / 1.0e6), -max_integral, max_integral);
  setMotor(u1, PWM1, IN1_1, IN2_1);

  float u2 = constrain(5 * (vt2 - v2Filt) + 10 * eintegral2, -255, 255);
  eintegral2 = constrain(eintegral2 + (vt2 - v2Filt) * ((micros() - prevT2) / 1.0e6), -max_integral, max_integral);
  setMotor(u2, PWM2, IN1_2, IN2_2);
}

void setMotor(float u, int pwm, int in1, int in2) {
  int dir = (u < 0) ? -1 : 1;
  int pwr = min((int)fabs(u), 255);
  analogWrite(pwm, pwr);
  digitalWrite(in1, dir == 1);
  digitalWrite(in2, dir != 1);
}

void readEncoder1() {
  pos_i1 += (digitalRead(ENCB1) ? 1 : -1);
}

void readEncoder2() {
  pos_i2 += (digitalRead(ENCB2) ? -1 : 1);
}

void clearSerialBuffer() {
  while (Serial.available() > 0) {
    Serial.read();
  }
}
