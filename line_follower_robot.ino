/*
 * ================================================================
 *  INTEGRATED LINE FOLLOWER + ROBOTIC ARM + COLOR DETECTION
 *  Platform  : ESP32
 *  Libraries : ESP32Servo, Adafruit_TCS34725, Wire
 * ----------------------------------------------------------------
 *  COLOR CALIBRATION — FULLY UPDATED (March 27 2026):
 *  RED   rN=0.5077  gN=0.4308  bN=0.3385
 *  GREEN rN=0.5375  gN=0.4788  bN=0.3648
 *
 *  SEPARATOR: (rN-gN) >= 0.070 = RED
 *             (rN-gN) <= 0.068 = GREEN
 *
 *  FIXES IN THIS VERSION:
 *  [1] OBJECT_STOP_CM = 6cm  — cleaner color reading distance
 *  [2] UNKNOWN color -> avoid — never drive into unidentified object
 *  [3] 7 samples, require 4/7 — more reliable majority vote
 *  [4] ignoreSharpTurnUntil set on turn COMPLETE — no false deposit
 *  [5] END_OF_PATH_MS = 1500ms — brief dropouts never trigger
 * ================================================================
 */

#include <Wire.h>
#include <ESP32Servo.h>
#include "Adafruit_TCS34725.h"

// --- MOTOR PINS ---
#define RIGHT_IN1  26
#define RIGHT_IN2  27
#define RIGHT_PWM  18
#define LEFT_IN1   14
#define LEFT_IN2   12
#define LEFT_PWM   19

// --- LINE SENSORS  (0 = on line, 1 = off line) ---
#define S1  34
#define S2  35
#define S3  32
#define S4  33
#define S5  25
#define S6  13
#define S7  15

// --- ULTRASONIC ---
#define TRIG_PIN  4
#define ECHO_PIN  5

// --- SERVO PINS ---
#define SERVO_ARM   16
#define SERVO_GRIP  17

// --- I2C ---
#define I2C_SDA  21
#define I2C_SCL  22

// --- HARDWARE OBJECTS ---
Servo armServo;
Servo gripServo;
Adafruit_TCS34725 tcs = Adafruit_TCS34725(
  TCS34725_INTEGRATIONTIME_50MS,
  TCS34725_GAIN_4X
);

// --- MOTOR PWM ---
const int PWM_FREQ       = 1000;
const int PWM_RESOLUTION = 8;

// --- SERVO POSITIONS ---
const int ARM_UP     = 180;
const int ARM_DOWN   = 3;
const int GRIP_OPEN  = 20;
const int GRIP_CLOSE = 80;

int armPos  = ARM_UP;
int gripPos = GRIP_OPEN;

// --- LINE FOLLOW PID ---
int   baseSpeed      = 110;
int   maxSpeed       = 130;
int   minSpeed       = -130;
int   sharpTurnSpeed = 135;

float Kp = 30.0;
float Ki = 0.0;
float Kd = 18.0;

float lineError  = 0;
float prevError  = 0;
float integral   = 0;
float derivative = 0;
float correction = 0;

// --- SHARP TURN STATE ---
bool          turning              = false;
int           turnDirection        = 0;
unsigned long turnStartTime        = 0;
unsigned long ignoreSharpTurnUntil = 0;
const unsigned long SHARP_IGNORE_MS = 800;

// --- OBJECT DETECTION ---
const float OBJECT_DETECT_CM = 12.0;
const float OBJECT_STOP_CM   = 6.0;   // FIX [1]: was 8cm — 6cm gives cleaner color reading
const float MIN_VALID_CM     = 2.0;
const float GREEN_PICKUP_CM  = 4.0;

unsigned long lastObjectAction = 0;
const unsigned long OBJECT_COOLDOWN_MS = 2500;

// --- COLOR CALIBRATION ---
float RED_REF_R   = 0.5077, RED_REF_G   = 0.4308, RED_REF_B   = 0.3385;
float GREEN_REF_R = 0.5375, GREEN_REF_G = 0.4788, GREEN_REF_B = 0.3648;

// --- rN-gN SEPARATOR ---
// RED   all readings: rN-gN = 0.077 to 0.509  (always >= 0.070)
// GREEN all readings: rN-gN = 0.052 to 0.066  (always <= 0.068)
// Gap 0.068-0.070 — zero overlap at any distance or lighting
const float RED_DIFF_THRESHOLD   = 0.070f;
const float GREEN_DIFF_THRESHOLD = 0.068f;
const float MAX_DIST_THRESHOLD   = 0.20f;

// FIX [3]: 7 samples, majority = 4 — more reliable vote
const int COLOR_SAMPLES      = 7;
const int COLOR_MAJORITY     = 4;

// --- RED AVOIDANCE TIMING ---
const int AVOID_BACK_SPEED  = -130;
const int AVOID_TURN_SPEED  =  130;
const int AVOID_FWD_SPEED   =  120;
const int REJOIN_L_SPEED    =  120;
const int REJOIN_R_SPEED    =   70;

const int AVOID_BACK_MS     =  250;
const int AVOID_TURN_R_MS   =  130;
const int AVOID_PASS_MS     =  550;
const int AVOID_TURN_L_MS   =  200;
const int AVOID_FWD2_MS     =  500;

// --- END OF PATH ---
unsigned long allSensorsOffSince   = 0;
const unsigned long END_OF_PATH_MS = 1500;  // FIX [5]: raised from 400ms

// --- POST PICKUP GUARD ---
unsigned long ignoreEndOfPathUntil = 0;
const unsigned long PICKUP_SETTLE_MS = 3000;

// --- OBJECT CARRY FLAG ---
bool holdingObject = false;

// --- FSM STATES ---
enum RobotState {
  STATE_LINE_FOLLOW,
  STATE_OBJECT_APPROACH,
  STATE_COLOR_CHECK,
  STATE_PICK_GREEN,
  STATE_SKIP_RED,
  STATE_RESUME_LINE,
  STATE_DEPOSIT_AND_STOP
};

RobotState robotState = STATE_LINE_FOLLOW;

// --- FUNCTION DECLARATIONS ---
void stateLineFollow();
void stateObjectApproach();
void stateColorCheck();
void statePickGreen();
void stateSkipRed();
void stateResumeLine();
void stateDepositAndStop();
void holdArmUp();
void pickGreenObject();
void skipRedObject();
void depositAndStop();
void runLineFollower();
int  readStableSensor(int pin);
float readDistanceCM();
void readColorRaw(uint16_t &r, uint16_t &g, uint16_t &b,
                  uint16_t &c, float &rN, float &gN, float &bN);
float colorDist(float r1, float g1, float b1,
                float r2, float g2, float b2);
String classifyColor(float rN, float gN, float bN, uint16_t c);
String detectColor();
void moveArmSlow(int target);
void moveArmFast(int target);
void moveGripSlow(int target);
void setMotorSpeeds(int L, int R);
void stopMotors();


// ================================================================
// SETUP
// ================================================================
void setup() {
  Serial.begin(115200);

  pinMode(RIGHT_IN1, OUTPUT);
  pinMode(RIGHT_IN2, OUTPUT);
  pinMode(LEFT_IN1,  OUTPUT);
  pinMode(LEFT_IN2,  OUTPUT);

  pinMode(S1,INPUT); pinMode(S2,INPUT); pinMode(S3,INPUT);
  pinMode(S4,INPUT); pinMode(S5,INPUT);
  pinMode(S6,INPUT); pinMode(S7,INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  Wire.begin(I2C_SDA, I2C_SCL);

  ledcAttach(LEFT_PWM,  PWM_FREQ, PWM_RESOLUTION);
  ledcAttach(RIGHT_PWM, PWM_FREQ, PWM_RESOLUTION);

  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  armServo.setPeriodHertz(50);
  gripServo.setPeriodHertz(50);

  armPos  = ARM_UP;
  gripPos = GRIP_OPEN;
  armServo.write(ARM_UP);
  gripServo.write(GRIP_OPEN);

  armServo.attach(SERVO_ARM,  500, 2400);
  gripServo.attach(SERVO_GRIP, 500, 2400);

  armServo.write(ARM_UP);
  gripServo.write(GRIP_OPEN);

  delay(1500);

  if (!tcs.begin()) {
    Serial.println("[ERROR] TCS34725 not found");
    while (1) delay(100);
  }

  stopMotors();

  Serial.println("=========================================");
  Serial.println(" ROBOT READY — ALL FIXES APPLIED");
  Serial.println(" Color check at: 6cm (was 8cm)");
  Serial.println(" UNKNOWN color:  AVOID (was resume)");
  Serial.println(" Samples: 7, majority: 4");
  Serial.println(" (rN-gN)>=0.070 -> RED  -> avoid");
  Serial.println(" (rN-gN)<=0.068 -> GREEN -> pick");
  Serial.println(" END_OF_PATH: 1500ms sustained");
  Serial.println(" Sharp turn guard: 800ms after complete");
  Serial.println("=========================================");
}


// ================================================================
// MAIN LOOP
// ================================================================
void loop() {
  switch (robotState) {
    case STATE_LINE_FOLLOW:      stateLineFollow();      break;
    case STATE_OBJECT_APPROACH:  stateObjectApproach();  break;
    case STATE_COLOR_CHECK:      stateColorCheck();      break;
    case STATE_PICK_GREEN:       statePickGreen();       break;
    case STATE_SKIP_RED:         stateSkipRed();         break;
    case STATE_RESUME_LINE:      stateResumeLine();      break;
    case STATE_DEPOSIT_AND_STOP: stateDepositAndStop();  break;
  }
}


// ================================================================
// STATE: LINE FOLLOW
// ================================================================
void stateLineFollow() {
  float d = readDistanceCM();

  if (d >= MIN_VALID_CM &&
      d <= OBJECT_DETECT_CM &&
      (millis() - lastObjectAction) > OBJECT_COOLDOWN_MS) {
    Serial.print("[DETECT] Object at ");
    Serial.print(d); Serial.println(" cm");
    stopMotors();
    delay(100);
    allSensorsOffSince = 0;
    robotState = STATE_OBJECT_APPROACH;
    return;
  }

  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);
  int s5 = digitalRead(S5);
  bool allOff = (s1==1 && s2==1 && s3==1 && s4==1 && s5==1);

  if (allOff &&
      holdingObject &&
      !turning &&
      (millis() > ignoreSharpTurnUntil) &&
      (millis() > ignoreEndOfPathUntil)) {

    if (allSensorsOffSince == 0) {
      allSensorsOffSince = millis();
      Serial.println("[CHECK] All sensors dark — timing 1500ms...");
    }

    if (millis() - allSensorsOffSince >= END_OF_PATH_MS) {
      Serial.println("[END] End of path confirmed — depositing");
      stopMotors();
      robotState = STATE_DEPOSIT_AND_STOP;
      return;
    }

  } else {
    allSensorsOffSince = 0;
  }

  runLineFollower();
}


// ================================================================
// STATE: OBJECT APPROACH — creep to 6cm for color check
// ================================================================
void stateObjectApproach() {
  unsigned long t0 = millis();

  while (millis() - t0 < 2000) {
    float d = readDistanceCM();

    if (d >= MIN_VALID_CM && d <= OBJECT_STOP_CM) {
      stopMotors();
      Serial.print("[APPROACH] Stopped at ");
      Serial.print(d); Serial.println(" cm — color check");
      delay(300);
      robotState = STATE_COLOR_CHECK;
      return;
    }

    setMotorSpeeds(75, 75);
    delay(25);
  }

  stopMotors();
  robotState = STATE_COLOR_CHECK;
}


// ================================================================
// STATE: COLOR CHECK
// FIX [2]: UNKNOWN now triggers AVOID — never drive into object
// ================================================================
void stateColorCheck() {
  stopMotors();
  delay(300);

  String color = detectColor();
  lastObjectAction = millis();

  Serial.print("[COLOR] Final decision: ");
  Serial.println(color);

  if (holdingObject) {
    if (color == "RED" || color == "GREEN" || color == "UNKNOWN") {
      Serial.println("[COLOR] Holding + obstacle -> avoid");
      robotState = STATE_SKIP_RED;
    } else {
      robotState = STATE_RESUME_LINE;
    }
    return;
  }

  if (color == "GREEN") {
    robotState = STATE_PICK_GREEN;
  }
  else if (color == "RED") {
    robotState = STATE_SKIP_RED;
  }
  else {
    // UNKNOWN — object is in front but unidentified
    // Safe behavior: AVOID rather than drive into it
    Serial.println("[COLOR] UNKNOWN — avoiding as safety measure");
    robotState = STATE_SKIP_RED;
  }
}


// ================================================================
// STATE: PICK GREEN
// ================================================================
void statePickGreen() {
  Serial.println("[ACTION] GREEN confirmed — creeping to 4cm");
  pickGreenObject();
  holdingObject        = true;
  ignoreEndOfPathUntil = millis() + PICKUP_SETTLE_MS;
  robotState           = STATE_RESUME_LINE;
}


// ================================================================
// STATE: SKIP RED
// ================================================================
void stateSkipRed() {
  Serial.println("[ACTION] Avoiding obstacle");
  holdArmUp();
  skipRedObject();
  robotState = STATE_LINE_FOLLOW;
}


// ================================================================
// STATE: RESUME LINE
// ================================================================
void stateResumeLine() {
  integral           = 0;
  prevError          = 0;
  lineError          = 0;
  correction         = 0;
  turning            = false;
  allSensorsOffSince = 0;
  Serial.println("[RESUME] Line following resumed");
  robotState = STATE_LINE_FOLLOW;
}


// ================================================================
// STATE: DEPOSIT AND STOP
// ================================================================
void stateDepositAndStop() {
  depositAndStop();
}


// ================================================================
// HOLD ARM UP — gripper stays CLOSED when carrying
// ================================================================
void holdArmUp() {
  armServo.write(ARM_UP);
  armPos = ARM_UP;

  if (!holdingObject) {
    gripServo.write(GRIP_OPEN);
    gripPos = GRIP_OPEN;
  }
}


// ================================================================
// PICK GREEN OBJECT
// Phase 1 — continuous creep from 6cm to 4cm
// Phase 2 — arm pickup sequence
// Phase 3 — reverse back onto line
// ================================================================
void pickGreenObject() {
  stopMotors();
  delay(300);

  const int CREEP_SPEED = 110;

  Serial.println("[ARM] Creeping from 6cm to 4cm...");

  setMotorSpeeds(CREEP_SPEED, CREEP_SPEED);

  unsigned long t0 = millis();
  while (millis() - t0 < 4000) {

    float d1 = readDistanceCM();
    delay(30);
    float d2 = readDistanceCM();

    float d = -1;
    if      (d1 >= MIN_VALID_CM && d2 >= MIN_VALID_CM) d = min(d1, d2);
    else if (d1 >= MIN_VALID_CM)                        d = d1;
    else if (d2 >= MIN_VALID_CM)                        d = d2;

    Serial.print("  dist: "); Serial.println(d);

    if (d > 0 && d <= GREEN_PICKUP_CM) {
      stopMotors();
      Serial.print("[ARM] At 4cm: "); Serial.println(d);
      break;
    }

    if (d > 0 && d < MIN_VALID_CM) {
      stopMotors();
      Serial.println("[ARM] Too close — stopping");
      break;
    }
  }

  stopMotors();
  delay(400);

  moveGripSlow(GRIP_OPEN);
  delay(200);
  moveArmSlow(ARM_DOWN);
  delay(300);
  moveGripSlow(GRIP_CLOSE);
  delay(500);
  moveArmFast(ARM_UP);
  delay(500);

  Serial.println("[ARM] Reversing onto line...");
  setMotorSpeeds(-100, -100);
  delay(600);
  stopMotors();
  delay(300);

  Serial.println("[ARM] Pickup complete");
}


// ================================================================
// DEPOSIT AND STOP
// ================================================================
void depositAndStop() {
  stopMotors();
  delay(500);

  Serial.println("[DEPOSIT] Depositing object...");

  moveArmSlow(ARM_DOWN);
  delay(300);
  moveGripSlow(GRIP_OPEN);
  delay(500);
  moveArmFast(ARM_UP);
  delay(500);

  holdingObject = false;
  stopMotors();

  Serial.println("=========================================");
  Serial.println("  ALL TASKS COMPLETE — ROBOT STOPPED");
  Serial.println("=========================================");

  while (true) { delay(1000); }
}


// ================================================================
// SKIP RED / AVOID OBSTACLE
// ================================================================
void skipRedObject() {
  setMotorSpeeds(AVOID_BACK_SPEED, AVOID_BACK_SPEED);
  delay(AVOID_BACK_MS);
  stopMotors(); delay(100);

  setMotorSpeeds(-AVOID_TURN_SPEED, AVOID_TURN_SPEED);
  delay(AVOID_TURN_R_MS);
  stopMotors(); delay(100);

  setMotorSpeeds(AVOID_FWD_SPEED, AVOID_FWD_SPEED);
  delay(AVOID_PASS_MS);
  stopMotors(); delay(100);

  setMotorSpeeds(AVOID_TURN_SPEED, -AVOID_TURN_SPEED);
  delay(AVOID_TURN_L_MS);
  stopMotors(); delay(100);

  setMotorSpeeds(AVOID_FWD_SPEED, AVOID_FWD_SPEED);
  delay(AVOID_FWD2_MS);
  stopMotors(); delay(80);

  unsigned long t0 = millis();
  while (millis() - t0 < 2500) {
    setMotorSpeeds(REJOIN_L_SPEED, REJOIN_R_SPEED);
    if (digitalRead(S3) == 0) {
      stopMotors();
      Serial.println("[AVOID] Line rejoined on S3");
      ignoreSharpTurnUntil = millis() + SHARP_IGNORE_MS;
      delay(100);
      return;
    }
    delay(10);
  }

  stopMotors();
  Serial.println("[AVOID] Warning: line rejoin timeout");
}


// ================================================================
// LINE FOLLOWER — PID + sharp turns
// FIX [4]: ignoreSharpTurnUntil set on turn COMPLETE
// ================================================================
void runLineFollower() {
  int s1 = digitalRead(S1);
  int s2 = digitalRead(S2);
  int s3 = digitalRead(S3);
  int s4 = digitalRead(S4);
  int s5 = digitalRead(S5);
  int s6 = readStableSensor(S6);
  int s7 = readStableSensor(S7);

  bool frontDetect      = (s1==0||s2==0||s3==0||s4==0||s5==0);
  bool sharpTurnAllowed = (millis() > ignoreSharpTurnUntil);

  if (sharpTurnAllowed && !turning && frontDetect && s6 == 0) {
    turning       = true;
    turnDirection = -1;
    turnStartTime = millis();
    setMotorSpeeds(baseSpeed, baseSpeed);
    delay(110);
  }

  if (sharpTurnAllowed && !turning && frontDetect && s7 == 0) {
    turning       = true;
    turnDirection = 1;
    turnStartTime = millis();
    setMotorSpeeds(baseSpeed, baseSpeed);
    delay(110);
  }

  if (turning) {
    if (turnDirection == -1) setMotorSpeeds(-sharpTurnSpeed,  sharpTurnSpeed);
    else                      setMotorSpeeds( sharpTurnSpeed, -sharpTurnSpeed);

    if (millis() - turnStartTime > 20 && digitalRead(S3) == 0) {
      turning = false;
      // FIX [4]: Block end-of-path detection for 800ms after turn completes
      // Prevents brief all-sensors-off after sharp turn triggering false deposit
      ignoreSharpTurnUntil = millis() + SHARP_IGNORE_MS;
    }
    return;
  }

  int   cnt = 0;
  float ws  = 0;

  if (s1==0){ ws +=  2.0f; cnt++; }
  if (s2==0){ ws +=  1.0f; cnt++; }
  if (s3==0){ ws +=  0.0f; cnt++; }
  if (s4==0){ ws += -1.0f; cnt++; }
  if (s5==0){ ws += -2.0f; cnt++; }

  if (cnt == 0) {
    int ls = constrain((int)(baseSpeed + correction), minSpeed, maxSpeed);
    int rs = constrain((int)(baseSpeed - correction), minSpeed, maxSpeed);
    setMotorSpeeds(ls, rs);
    return;
  }

  lineError  = ws / cnt;
  integral   = constrain(integral + lineError, -200.0f, 200.0f);
  derivative = lineError - prevError;
  correction = Kp * lineError + Ki * integral + Kd * derivative;
  prevError  = lineError;

  int dynBase    = constrain(baseSpeed - (int)(fabsf(lineError) * 10), 60, baseSpeed);
  int leftSpeed  = constrain(dynBase + (int)correction, minSpeed, maxSpeed);
  int rightSpeed = constrain(dynBase - (int)correction, minSpeed, maxSpeed);

  setMotorSpeeds(leftSpeed, rightSpeed);
}

int readStableSensor(int pin) {
  int low = 0;
  for (int i = 0; i < 5; i++) {
    if (digitalRead(pin) == 0) low++;
    delayMicroseconds(800);
  }
  return (low >= 4) ? 0 : 1;
}


// ================================================================
// ULTRASONIC
// ================================================================
float readDistanceCM() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(3);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long dur = pulseIn(ECHO_PIN, HIGH, 30000);
  if (dur == 0) return -1.0f;
  return dur * 0.0343f / 2.0f;
}


// ================================================================
// COLOR DETECTION
// ================================================================
void readColorRaw(uint16_t &r, uint16_t &g, uint16_t &b,
                  uint16_t &c, float &rN, float &gN, float &bN) {
  tcs.getRawData(&r, &g, &b, &c);
  if (c > 0) {
    rN = (float)r / c;
    gN = (float)g / c;
    bN = (float)b / c;
  } else {
    rN = gN = bN = 0;
  }
}

float colorDist(float r1, float g1, float b1,
                float r2, float g2, float b2) {
  float dr=r1-r2, dg=g1-g2, db=b1-b2;
  return sqrtf(dr*dr + dg*dg + db*db);
}

// ================================================================
// classifyColor — rN-gN SEPARATOR (primary) + distance fallback
//
// At 6cm stop distance:
//   RED   rN-gN ~ 0.10-0.15 -> well above 0.070 threshold
//   GREEN rN-gN ~ 0.05-0.06 -> well below 0.068 threshold
//   Gap is wider at 6cm than 8cm — cleaner classification
// ================================================================
String classifyColor(float rN, float gN, float bN, uint16_t c) {
  if (c < 40) return "UNKNOWN";

  float diff = rN - gN;

  Serial.print("    diff(rN-gN)="); Serial.println(diff, 4);

  if (diff >= RED_DIFF_THRESHOLD)   return "RED";
  if (diff <= GREEN_DIFF_THRESHOLD) return "GREEN";

  // Gap zone 0.068-0.070 — use distance fallback
  float dR = colorDist(rN, gN, bN, RED_REF_R,   RED_REF_G,   RED_REF_B);
  float dG = colorDist(rN, gN, bN, GREEN_REF_R, GREEN_REF_G, GREEN_REF_B);
  float minD = min(dR, dG);
  if (minD > MAX_DIST_THRESHOLD) return "UNKNOWN";
  return (dR < dG) ? "RED" : "GREEN";
}

// FIX [3]: 7 samples, require 4/7 for reliable majority
String detectColor() {
  int redCnt = 0, greenCnt = 0;

  for (int i = 0; i < COLOR_SAMPLES; i++) {
    uint16_t r, g, b, c;
    float rN, gN, bN;
    readColorRaw(r, g, b, c, rN, gN, bN);
    String col = classifyColor(rN, gN, bN, c);

    Serial.print("  s"); Serial.print(i+1);
    Serial.print(": "); Serial.print(col);
    Serial.print("  rN="); Serial.print(rN, 4);
    Serial.print(" gN="); Serial.print(gN, 4);
    Serial.print(" diff="); Serial.println(rN - gN, 4);

    if      (col == "RED")   redCnt++;
    else if (col == "GREEN") greenCnt++;

    delay(60);
  }

  Serial.print("[VOTE] RED="); Serial.print(redCnt);
  Serial.print(" GREEN="); Serial.print(greenCnt);
  Serial.print(" need "); Serial.println(COLOR_MAJORITY);

  if (redCnt   >= COLOR_MAJORITY) return "RED";
  if (greenCnt >= COLOR_MAJORITY) return "GREEN";
  return "UNKNOWN";
}


// ================================================================
// SERVO HELPERS
// ================================================================
void moveArmSlow(int target) {
  target = constrain(target, 0, 180);
  int stp = (target > armPos) ? 1 : -1;
  while (armPos != target) {
    armPos += stp;
    armServo.write(armPos);
    delay(15);
  }
}

void moveArmFast(int target) {
  target = constrain(target, 0, 180);
  int stp = (target > armPos) ? 2 : -2;
  while (abs(target - armPos) > 1) {
    armPos += stp;
    armServo.write(armPos);
    delay(5);
  }
  armPos = target;
  armServo.write(armPos);
}

void moveGripSlow(int target) {
  target = constrain(target, 0, 180);
  int stp = (target > gripPos) ? 1 : -1;
  while (gripPos != target) {
    gripPos += stp;
    gripServo.write(gripPos);
    delay(15);
  }
}


// ================================================================
// MOTOR CONTROL
// ================================================================
void setMotorSpeeds(int L, int R) {
  L = constrain(L, -255, 255);
  R = constrain(R, -255, 255);

  if      (L > 0) { digitalWrite(LEFT_IN1,HIGH); digitalWrite(LEFT_IN2,LOW);  ledcWrite(LEFT_PWM,  L); }
  else if (L < 0) { digitalWrite(LEFT_IN1,LOW);  digitalWrite(LEFT_IN2,HIGH); ledcWrite(LEFT_PWM, -L); }
  else            { digitalWrite(LEFT_IN1,LOW);  digitalWrite(LEFT_IN2,LOW);  ledcWrite(LEFT_PWM,  0); }

  if      (R > 0) { digitalWrite(RIGHT_IN1,LOW);  digitalWrite(RIGHT_IN2,HIGH); ledcWrite(RIGHT_PWM,  R); }
  else if (R < 0) { digitalWrite(RIGHT_IN1,HIGH); digitalWrite(RIGHT_IN2,LOW);  ledcWrite(RIGHT_PWM, -R); }
  else            { digitalWrite(RIGHT_IN1,LOW);  digitalWrite(RIGHT_IN2,LOW);  ledcWrite(RIGHT_PWM,  0); }
}

void stopMotors() {
  setMotorSpeeds(0, 0);
}
