#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>

#define SDA_PIN 8
#define SCL_PIN 9

Adafruit_PWMServoDriver pca(0x40);
Adafruit_PWMServoDriver pca(0x41);

#define SERVO_MIN 110
#define SERVO_MAX 500

// ====== CẤU HÌNH SERVO ======
#define SERVO_S1 0   // háng
#define SERVO_S2 1   // đùi
#define SERVO_S3 2   // cẳng

// ====== BẢNG CHƯƠNG TRÌNH BƯỚC ======
const int stepTable[][3] = {
  { 90,  90, 180 },   // stand
  { 90, 120, 120 },
  {135, 120, 120 },
  {135,  90, 180 },
  { 45,  90, 180 }
};

const int STEP_COUNT = sizeof(stepTable) / sizeof(stepTable[0]);



// ====== HÀM CHUYỂN GÓC → PWM ======
int angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVO_MIN, SERVO_MAX);
}

// ====== HÀM SET 1 SERVO ======
void setServo(uint8_t ch, int angle) {
  pca.setPWM(ch, 0, angleToPulse(angle));
}

// ====== HÀM CHẠY 1 STEP ======
void runStep(int s1, int s2, int s3) {
  setServo(SERVO_S1, s1);
  setServo(SERVO_S2, s2);
  setServo(SERVO_S3, s3);
}

// ====== HÀM CHẠY TOÀN BỘ CHƯƠNG TRÌNH ======
void runSequence(int delayMs) {
  for (int i = 0; i < STEP_COUNT; i++) {
    runStep(
      stepTable[i][0],
      stepTable[i][1],
      stepTable[i][2]
    );
    delay(delayMs);
  }
}

void setup() {
  Wire.begin(SDA_PIN, SCL_PIN);

  pca.begin();
  pca.setPWMFreq(50);
  delay(500);

  // về tư thế đứng
  runStep(90, 90, 180);
  delay(1000);
}

void loop() {
  runSequence(500);   // chạy chương trình bước
  delay(1000);        // nghỉ giữa các chu kỳ
}
