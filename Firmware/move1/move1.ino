#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

/* =====================================================
   I. CẤU HÌNH PHẦN CỨNG
===================================================== */

// ESP32-C3 I2C
#define SDA_PIN 6
#define SCL_PIN 7

// Địa chỉ PCA9685
#define ADDR_RIGHT 0x40
#define ADDR_LEFT  0x60

// Servo
#define SERVO_FREQ 50
#define USMIN  600
#define USMAX  2400

/* =====================================================
   II. MAP CHANNEL PCA9685 (0–15)
===================================================== */

// Right Driver (0x40)
const int PIN_FR[3] = {0, 1, 2};
const int PIN_MR[3] = {3, 4, 5};
const int PIN_BR[3] = {6, 7, 8};

// Left Driver (0x60)
const int PIN_FL[3] = {0, 1, 2};
const int PIN_ML[3] = {3, 4, 5};
const int PIN_BL[3] = {6, 7, 8};

// PCA objects
Adafruit_PWMServoDriver pwmRight(ADDR_RIGHT);
Adafruit_PWMServoDriver pwmLeft(ADDR_LEFT);

/* =====================================================
   III. HÀM HỖ TRỢ
===================================================== */

int angleToPulse(float angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, USMIN, USMAX);
}

void setLeg(char side, const int pin[3], float c, float f, float t) {
  float s1, s2, s3;

  // Mirror logic
  if (side == 'R') {
    s1 = 90 + c;
    s2 = 90 + f;
    s3 = 90 - t;
  } else {
    s1 = 90 - c;
    s2 = 90 - f;
    s3 = 90 + t;
  }

  Adafruit_PWMServoDriver* drv = (side == 'R') ? &pwmRight : &pwmLeft;

  drv->writeMicroseconds(pin[0], angleToPulse(s1));
  drv->writeMicroseconds(pin[1], angleToPulse(s2));
  drv->writeMicroseconds(pin[2], angleToPulse(s3));
}

void applyAllLegs(float c, float f, float t) {
  setLeg('R', PIN_FR, c, f, t);
  setLeg('R', PIN_MR, c, f, t);
  setLeg('R', PIN_BR, c, f, t);
  setLeg('L', PIN_FL, c, f, t);
  setLeg('L', PIN_ML, c, f, t);
  setLeg('L', PIN_BL, c, f, t);
}

/* =====================================================
   IV. TEST TỪNG SERVO
===================================================== */

void testAllServos() {
  Serial.println(">>> TEST ALL SERVOS <<<");

  const int* rightLegs[3] = {PIN_FR, PIN_MR, PIN_BR};
  const int* leftLegs[3]  = {PIN_FL, PIN_ML, PIN_BL};

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print("Right leg "); Serial.print(i);
      Serial.print(" servo "); Serial.println(j);

      pwmRight.writeMicroseconds(rightLegs[i][j], angleToPulse(90));
      delay(500);
      pwmRight.writeMicroseconds(rightLegs[i][j], angleToPulse(30));
      delay(500);
      pwmRight.writeMicroseconds(rightLegs[i][j], angleToPulse(150));
      delay(500);
    }
  }

  for (int i = 0; i < 3; i++) {
    for (int j = 0; j < 3; j++) {
      Serial.print("Left leg "); Serial.print(i);
      Serial.print(" servo "); Serial.println(j);

      pwmLeft.writeMicroseconds(leftLegs[i][j], angleToPulse(90));
      delay(500);
      pwmLeft.writeMicroseconds(leftLegs[i][j], angleToPulse(30));
      delay(500);
      pwmLeft.writeMicroseconds(leftLegs[i][j], angleToPulse(150));
      delay(500);
    }
  }

  Serial.println(">>> TEST DONE <<<");
}

/* =====================================================
   V. SETUP
===================================================== */

void setup() {
  Serial.begin(115200);
  delay(3000);

  Serial.println("\n=== HEXAPOD SERVO TEST MODE ===");

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  delay(100);
  Serial.println("[OK] I2C started");

  // PCA9685 Right
  pwmRight.begin();
  pwmRight.setOscillatorFrequency(27000000);
  pwmRight.setPWMFreq(SERVO_FREQ);
  delay(10);

  // PCA9685 Left
  pwmLeft.begin();
  pwmLeft.setOscillatorFrequency(27000000);
  pwmLeft.setPWMFreq(SERVO_FREQ);
  delay(10);

  Serial.println("--------------------------------");
  Serial.println("NHẬP:");
  Serial.println("  C F T   (vd: 0 45 55)");
  Serial.println("  999     (test tung servo)");
  Serial.println("--------------------------------");

  applyAllLegs(0, 45, 55);
}

/* =====================================================
   VI. LOOP
===================================================== */

void loop() {
  if (Serial.available()) {

    float v1 = Serial.parseFloat();
    if (v1 == 999) {
      while (Serial.available()) Serial.read();
      testAllServos();
      return;
    }

    float v2 = Serial.parseFloat();
    float v3 = Serial.parseFloat();
    while (Serial.available()) Serial.read();

    Serial.print("Set All -> C:");
    Serial.print(v1);
    Serial.print(" F:");
    Serial.print(v2);
    Serial.print(" T:");
    Serial.println(v3);

    applyAllLegs(v1, v2, v3);
  }
}
