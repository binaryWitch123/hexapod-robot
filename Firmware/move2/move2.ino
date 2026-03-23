#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h> // Để dùng hàm isnan (Is Not A Number)

/* =====================================================
   I. CẤU HÌNH PHẦN CỨNG
===================================================== */
#define SDA_PIN 6
#define SCL_PIN 7
#define ADDR_RIGHT 0x40
#define ADDR_LEFT  0x60
#define SERVO_FREQ 50
#define USMIN  600
#define USMAX  2400

/* =====================================================
   II. PIN MAPPING
===================================================== */
const int PIN_FR[3] = {8, 9, 10}; 
const int PIN_MR[3] = {4, 5, 6};  
const int PIN_BR[3] = {0, 1, 2};  

const int PIN_FL[3] = {7, 6, 5};  
const int PIN_ML[3] = {11, 10, 9}; 
const int PIN_BL[3] = {15, 14, 13}; 

/* =====================================================
   III. CONFIG CHIỀU QUAY (DIR)
===================================================== */
int DIR_FR[3] = { -1,  -1,  -1};
int DIR_MR[3] = { -1,  -1,  -1};
int DIR_BR[3] = { -1,  -1,  -1};

int DIR_FL[3] = { 1, 1, 1};
int DIR_ML[3] = { 1, 1, 1};
int DIR_BL[3] = { 1, 1, 1};

/* =====================================================
   IV. CONFIG TINH CHỈNH (OFFSET)
===================================================== */
// --- BÊN PHẢI ---
int OFF_FR[3] = { 0,  0,  0}; 
int OFF_MR[3] = { 0,  0,  0}; 
int OFF_BR[3] = { 0,  0,  0}; 

// --- BÊN TRÁI ---
int OFF_FL[3] = { 0,  0,  0}; 
int OFF_ML[3] = { 0,  0,  0}; 
int OFF_BL[3] = { 0,  0,  0}; 

// Khởi tạo Driver
Adafruit_PWMServoDriver pwmRight(ADDR_RIGHT);
Adafruit_PWMServoDriver pwmLeft(ADDR_LEFT);

/* =====================================================
   V. HÀM XỬ LÝ CHÍNH
===================================================== */

int angleToPulse(float angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, USMIN, USMAX);
}

// Cập nhật hàm setLeg: Thêm kiểm tra NAN để bỏ qua
void setLeg(char side, const int pin[3], const int dir[3], const int offset[3], float c, float f, float t) {
  Adafruit_PWMServoDriver* drv = (side == 'R') ? &pwmRight : &pwmLeft;
  float angle_val;

  // --- SERVO 1: COXA ---
  // Nếu c KHÔNG PHẢI là NAN thì mới điều khiển
  if (!isnan(c)) { 
    angle_val = 90 + (c * dir[0]) + offset[0];
    drv->writeMicroseconds(pin[0], angleToPulse(angle_val));
  }

  // --- SERVO 2: FEMUR ---
  if (!isnan(f)) {
    angle_val = 90 + (f * dir[1]) + offset[1];
    drv->writeMicroseconds(pin[1], angleToPulse(angle_val));
  }

  // --- SERVO 3: TIBIA ---
  if (!isnan(t)) {
    angle_val = 90 + (t * dir[2]) + offset[2];
    drv->writeMicroseconds(pin[2], angleToPulse(angle_val));
  }
}

void applyAllLegs(float c, float f, float t) {
  setLeg('R', PIN_FR, DIR_FR, OFF_FR, c, f, t);
  setLeg('R', PIN_MR, DIR_MR, OFF_MR, c, f, t);
  setLeg('R', PIN_BR, DIR_BR, OFF_BR, c, f, t);
  
  setLeg('L', PIN_FL, DIR_FL, OFF_FL, c, f, t);
  setLeg('L', PIN_ML, DIR_ML, OFF_ML, c, f, t);
  setLeg('L', PIN_BL, DIR_BL, OFF_BL, c, f, t);
}

// Hàm tách chuỗi thông minh (Parse Input)
float parseToken(String token) {
  token.trim(); // Xóa khoảng trắng thừa
  if (token.equalsIgnoreCase("s")) {
    return NAN; // Trả về giá trị Not-A-Number đặc biệt
  }
  return token.toFloat();
}

/* =====================================================
   VI. SETUP & LOOP
===================================================== */
void setup() {
  Serial.begin(115200);
  Wire.begin(SDA_PIN, SCL_PIN);
  
  pwmRight.begin();
  pwmRight.setPWMFreq(SERVO_FREQ);
  
  pwmLeft.begin();
  pwmLeft.setPWMFreq(SERVO_FREQ);
  
  delay(100);
  
  Serial.println("HEXAPOD COMMAND MODE");
  Serial.println("[ANGLE -90 -> 90], [s - sleep], ");
  Serial.println("Type: [Angle1] [Angle2] [Angle3]");

  
  // Khởi động về 0 hết
  applyAllLegs(0, 0, 0);
}

void loop() {
  if (Serial.available()) {
    // Đọc cả dòng lệnh (vd: "0 s 45")
    String input = Serial.readStringUntil('\n');
    input.trim();

    if (input.length() > 0) {
      float values[3] = {NAN, NAN, NAN}; // Mặc định là Skip hết
      
      int space1 = input.indexOf(' ');
      int space2 = input.indexOf(' ', space1 + 1);

      // Tách chuỗi thủ công để lấy 3 phần
      if (space1 != -1) {
        String s1 = input.substring(0, space1);
        values[0] = parseToken(s1);

        if (space2 != -1) {
          String s2 = input.substring(space1 + 1, space2);
          String s3 = input.substring(space2 + 1);
          values[1] = parseToken(s2);
          values[2] = parseToken(s3);
        } else {
          // Trường hợp chỉ nhập 2 số (vd: "0 45") -> cái cuối skip
          String s2 = input.substring(space1 + 1);
          values[1] = parseToken(s2);
        }
      } else {
        // Trường hợp chỉ nhập 1 số (vd: "0") -> 2 cái sau skip
        values[0] = parseToken(input);
      }

      // In ra Debug để kiểm tra
      Serial.print("Lenh: ");
      if (isnan(values[0])) Serial.print("[Giữ] | "); else { Serial.print(values[0]); Serial.print(" | "); }
      if (isnan(values[1])) Serial.print("[Giữ] | "); else { Serial.print(values[1]); Serial.print(" | "); }
      if (isnan(values[2])) Serial.println("[Giữ]"); else Serial.println(values[2]);

      // Gửi lệnh xuống Driver
      applyAllLegs(values[0], values[1], values[2]);
    }
  }
}