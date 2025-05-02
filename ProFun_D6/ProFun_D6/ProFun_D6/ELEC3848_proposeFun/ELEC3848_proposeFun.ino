#include <String.h>
#include <FacialExpression.h>
#include <Wire.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 9   //Motor C PWM
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction




// Motor control macros
#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define SERIAL  Serial
#define BTSERIAL Serial3

#define LOG_DEBUG

#ifdef LOG_DEBUG
  #define M_LOG SERIAL.print
  #define M_LOGLN SERIAL.println
#else
  #define M_LOG BTSERIAL.print
  #define M_LOGLN BTSERIAL.println
#endif

// PWM Definition
#define MAX_PWM   255
#define MIN_PWM   100
#define NAV_PWM   200  // Navigation speed - lower for precision
#define MEDIUM_PWM 150 // Medium speed for controlled approach
#define FINE_PWM  120 // Fine adjustment speed
#define ULTRA_FINE_PWM 80 // Very slow for final positioning

// State machine states

// Global variables
Adafruit_MPU6050 mpu;
String inputString = "";
bool stringComplete = false, trackingMode = false;
unsigned long timer_gyro = 0;
int lastNumber = -1;
int numberCount = 0;

int buttonLeftPin = 48;
int buttonRightPin = 47;
int buttonBackPin = 33;
float targetYaw = 0;
bool turning = false;
float yaw;
unsigned long lastSerialCheck = 0, lastGyroCheck = 0, lastFaceUpdate = 0;
const unsigned long serialCheckInterval = 3;
// Movement functions
//    ↑A-----B↑
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void BACK(uint8_t pwm = 70)
{
  MOTORA_BACKOFF(pwm); 
  MOTORB_FORWARD(pwm);
  MOTORC_BACKOFF(pwm); 
  MOTORD_FORWARD(pwm);
}

//    ↓A-----B↓
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void ADVANCE(uint8_t pwm = 70)
{
  MOTORA_FORWARD(pwm+15); 
  MOTORB_BACKOFF(pwm);
  MOTORC_FORWARD(pwm); 
  MOTORD_BACKOFF(pwm);
}

//    =A-----B↑
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1(uint8_t pwm = 70)
{
  MOTORA_STOP(pwm); 
  MOTORB_FORWARD(pwm);
  MOTORC_BACKOFF(pwm); 
  MOTORD_STOP(pwm);
}

//    ↓A-----B↑
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void RIGHT_2(uint8_t pwm = 70)
{
  MOTORA_FORWARD(pwm + 15); 
  MOTORB_FORWARD(pwm);
  MOTORC_BACKOFF(pwm + 10); 
  MOTORD_BACKOFF(pwm + 15);
}

//    ↓A-----B=
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3(uint8_t pwm = 70)
{
  MOTORA_FORWARD(pwm); 
  MOTORB_STOP(pwm);
  MOTORC_STOP(pwm); 
  MOTORD_BACKOFF(pwm);
}

//    ↑A-----B=
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1(uint8_t pwm = 70)
{
  MOTORA_BACKOFF(pwm); 
  MOTORB_STOP(pwm);
  MOTORC_STOP(pwm); 
  MOTORD_FORWARD(pwm);
}

//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void LEFT_2(uint8_t pwm = 70)
{
  MOTORA_BACKOFF(pwm+10); 
  MOTORB_BACKOFF(pwm);
  MOTORC_FORWARD(pwm+10); 
  MOTORD_FORWARD(pwm);
}

//    =A-----B↓
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3(uint8_t pwm = 70)
{
  MOTORA_STOP(pwm); 
  MOTORB_BACKOFF(pwm);
  MOTORC_FORWARD(pwm); 
  MOTORD_STOP(pwm);
}

//    ↑A-----B↓
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
void rotate_1(uint8_t pwm = 70)  // Clockwise rotation(left)
{
  MOTORA_BACKOFF(pwm); 
  MOTORB_BACKOFF(pwm);
  MOTORC_BACKOFF(pwm); 
  MOTORD_BACKOFF(pwm);
}

//    ↓A-----B↑
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
void rotate_2(uint8_t pwm = 70)  // Counter-clockwise rotation(right)
{
  MOTORA_FORWARD(pwm);
  MOTORB_FORWARD(pwm);
  MOTORC_FORWARD(pwm);
  MOTORD_FORWARD(pwm);
}

// Stop all motors
void STOP()
{
  MOTORA_STOP(70);
  MOTORB_STOP(70);
  MOTORC_STOP(70);
  MOTORD_STOP(70);
}
FacialExpression face;
void setup() {
  SERIAL.begin(115200); // USB serial setup
  SERIAL.println("Parking Robot Starting");
  face.begin();
  Wire.begin();
  SERIAL.println("face done");
  face.fact("normal");
  Serial.println("set normal face");
  // Initialize motor control pins
  pinMode(PWMA, OUTPUT);
  pinMode(DIRA1, OUTPUT);
  pinMode(DIRA2, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(DIRB1, OUTPUT);
  pinMode(DIRB2, OUTPUT);
  pinMode(PWMC, OUTPUT);
  pinMode(DIRC1, OUTPUT);
  pinMode(DIRC2, OUTPUT);
  pinMode(PWMD, OUTPUT);
  pinMode(DIRD1, OUTPUT);
  pinMode(DIRD2, OUTPUT);


  if (!mpu.begin()) {
    //Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  SERIAL.println("mpu done");

  pinMode(buttonLeftPin, INPUT_PULLUP);
  pinMode(buttonRightPin, INPUT_PULLUP);
  pinMode(buttonBackPin, INPUT_PULLUP);
  


  //Serial.println("MPU6050 Found!");
  
}


int trackingStartCount = 0;
int trackingStopCount = 0;

void loop() {
  if (millis() - lastFaceUpdate > 100){
    face.update();
    lastFaceUpdate = millis();
  }
  unsigned long currentMillis = millis();

  if (currentMillis - lastSerialCheck >= serialCheckInterval) {
    lastSerialCheck = currentMillis;
    readLatestSerial();
  }

  if (stringComplete) {
    processInput(inputString);
    Serial.println("Got latest: " + inputString);

    // Clear after processing
    inputString = "";
    stringComplete = false;
  }

  if(!digitalRead(buttonLeftPin) || !digitalRead(buttonRightPin) || !digitalRead(buttonBackPin) || turning) { 
    GyroTurnControl_update();
  
  }
}


void readLatestSerial() {
  // Step 1: If too much junk, clear buffer
  if (Serial.available() > 64) { // or even >32 if you want stricter
    while (Serial.available()) Serial.read();
    inputString = "";
    stringComplete = false;
    return; // skip reading this cycle
  }

  // Step 2: Read incoming characters
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\r') {
      // ignore carriage return
    } 
    else if (inChar == '\n') {
      stringComplete = true;
      break; // message complete
    } 
    else {
      inputString += inChar; // keep building the message
    }
  }
}

void processInput(String data) {
  int curr_count = extractValue(data, "curr_count");
  int cx = extractValue(data, "cx");
  int height = extractValue(data, "height");
  
  bool followMode = extractBoolValue(data, "follow_mode"); // NEW line to get follow_mode

  if (!trackingMode) {
    STOP();
    if (followMode) {  // Modified to also check followMode

      trackingMode = true;
      face.fact("heart");
      Serial.println("Tracking started");
      }
    else {
      handleNumber(curr_count);
    }
  } else {
    if (!followMode) {  // Only stop if followMode is false
      trackingMode = false;
      Serial.println("Tracking stopped");
      face.fact("normal");
      STOP();
      trackingStopCount = 0;
      return;
      }

    // Tracking logic
    if (10 < cx && cx < 135) {
      rotate_1(80);  // LEFT
      Serial.println("LEFT");
      delay(35);
      STOP();
    } else if (cx > 210) {
      rotate_2(80);  // RIGHT
      Serial.println("Right");
      delay(35);
      STOP();
    } else {
      if (10 < height && height < 150) {
        ADVANCE(50);
        Serial.println("ADVANCE");
      } else if (height > 210) {
        BACK(50);
      } else {
        STOP();
        Serial.println("stop");
      }
    }
  }
}

int extractValue(String data, String key) {
  int idx = data.indexOf(key);
  if (idx == -1) return 0;  // Key not found
  int colonIdx = data.indexOf(':', idx);
  int commaIdx = data.indexOf(',', colonIdx);
  if (commaIdx == -1) {
    commaIdx = data.indexOf('}', colonIdx);  // Last field
  }
  String valueStr = data.substring(colonIdx + 1, commaIdx);
  valueStr.trim();
  return valueStr.toInt();
}

bool extractBoolValue(String data, String key) {
  int idx = data.indexOf(key);
  if (idx == -1) return false;  // Key not found
  int colonIdx = data.indexOf(':', idx);
  int commaIdx = data.indexOf(',', colonIdx);
  if (commaIdx == -1) {
    commaIdx = data.indexOf('}', colonIdx);  // Last field
  }
  String valueStr = data.substring(colonIdx + 1, commaIdx);
  valueStr.trim();
  return (valueStr == "True" || valueStr == "true");
}


void handleNumber(int number) {
  if (number < 0 || number > 5) {
    //Serial.println("Invalid number");
    return;
  }

  if (number == lastNumber) {
    numberCount++;
  } else {
    numberCount = 1;
    lastNumber = number;
  }

  if (numberCount >= 5) {
    switch (number) {
      case 0:
        break;

      case 1:
        Serial.println("tracking\n");
        dance();
        break;

      case 2:
        spinCircle();
        break;

      case 3:
        face.fact("sleepy");
        STOP();
        break;

      case 4:
        diagonalHop();
        break;

      
      default:
        break;
    }
    numberCount = 0;
    lastNumber = -1;
  }

  //Serial.print("Received number: ");
  //Serial.println(number);
}

void dance() {
  // 第一次动作
  face.fact("heart");  // 爱心眼
  rotate_1(70);        // 左转
  delay(400);

  // 第二次动作
  face.fact("cute");   // 可爱眼
  rotate_2(70);        // 右转
  delay(400);

  // 第三次动作
  face.fact("angry");  // 生气眼
  ADVANCE(80);         // 前冲
  delay(300);

  // 第四次动作
  face.fact("sleepy"); // 困了眼
  BACK(80);            // 后退
  delay(300);

  // 第五次动作
  face.fact("x_x"); // 
  LEFT_2(70);          // 左侧滑
  delay(400);

  face.fact("cute");   // 结束时可爱
  STOP();  
  face.fact("normal");            // 最后停下来
}

void spinCircle() {
  face.fact("cute");     // 切换成可爱表情（转圈嘛，当然要可爱啦）
  
  rotate_1(100);         // 原地顺时针转圈（转得快一点，像跳舞）
  
  delay(2500);           // 转 3 秒钟

  STOP();                // 别忘了转完停下来
  face.fact("normal");   // 转完恢复正常表情
}

void diagonalHop() {
  // 往左上跳
  face.fact("cute");
  LEFT_1(100); // 左上方向快走
  delay(300); // 跳一下
  STOP();
  delay(100); // 小停顿

  // 往右上跳
  face.fact("heart");
  RIGHT_1(100);
  delay(300);
  STOP();
  delay(100);

  // 往左下跳
  face.fact("squint");
  LEFT_3(100);
  delay(300);
  STOP();
  delay(100);

  // 往右下跳
  face.fact("angry");
  RIGHT_3(100);
  delay(300);
  STOP();
  delay(100);

  // 跳完回到正常表情
  face.fact("normal");
}

//-----------------------------------------------------// Gyroscop



void GyroTurnControl_update() {
  if (millis() - lastGyroCheck < 200) {
    return;  // Correct way to exit function early
  }
  
  lastGyroCheck = millis();

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);  // Get accelerometer, gyro, and temp data
  Serial.println("get Gyro");
  static unsigned long lastTime = 0;
  unsigned long now = millis();

  if (lastTime == 0) {
    lastTime = now;  // Initialize lastTime first call
    return;  // Not enough data to calculate dt
  }

  float dt = (now - lastTime) / 1000.0;  // dt in seconds
  lastTime = now;

  float gyroZ = g.gyro.z * 180.0 / PI;  // radians/sec -> degrees/sec
  yaw += gyroZ * dt;
  yaw = normalizeYaw(yaw);

  if (!digitalRead(buttonLeftPin) && !turning) {
    targetYaw = normalizeYaw(yaw + 90);
    turning = true;
    rotate_1(70);
    face.fact("right");
  }
  
  if (!digitalRead(buttonRightPin) && !turning) {
    targetYaw = normalizeYaw(yaw - 90);
    turning = true;
    rotate_2(70);
    face.fact("left");
  }
  
  if (!digitalRead(buttonBackPin) && !turning) {
    targetYaw = normalizeYaw(yaw - 180);
    turning = true;
    rotate_2(70);
    face.fact("left");
  }

  if (turning) {
    if (isTargetReached(yaw, targetYaw)) {
      STOP();
      turning = false;
      face.fact("normal");
    }
  }
}


float normalizeYaw(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
}

bool isTargetReached(float current, float target) {
  float diff = normalizeYaw(target - current);
  Serial.println("check target");
  //Serial.println(diff);
  return abs(diff) < 10.0; // 10 degree threshold
}

