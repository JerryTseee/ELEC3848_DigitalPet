#include "FacialExpression.h"

FacialExpression::FacialExpression() : display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET) {}

void FacialExpression::begin() {
  if (!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    while (true);
  }
  display.clearDisplay();
  display.display();
}

void FacialExpression::fact(String expression) {
  EyeDirection currentExpression;
  
  if (expression == "cute") {
    currentExpression = CUTE;
    isNormalMode = false;
  } else if (expression == "sleepy") {
    currentExpression = SLEEPY;
    isNormalMode = false;
  } else if (expression == "angry") {
    currentExpression = ANGRY;
    isNormalMode = false;
  } else if (expression == "heart") {
    currentExpression = HEART;
    isNormalMode = false;
  } else if (expression == "x_x") {
    currentExpression = X_X;
    isNormalMode = false;
  } else if (expression == "squint") {
    currentExpression = SQUINT;
    isNormalMode = false;
  } else if (expression == "left") {
    currentExpression = LEFT;
    isNormalMode = false;
  } else if (expression == "right") {
    currentExpression = RIGHT;
    isNormalMode = false;
  } else if (expression == "normal") { 
    isNormalMode = true; // 进入normal模式
    lastUpdateTime = millis(); // 记录时间
    drawEyes(currentNormalDirection); // 画当前normal方向
    return;
  } else {
    currentExpression = CENTER;
    isNormalMode = false;
  }

  drawEyes(currentExpression);
}

void FacialExpression::drawEyes(EyeDirection dir) {
  display.clearDisplay();
  switch (dir) {
    case CUTE: drawCuteEyes(); break;
    case ANGRY: drawAngryEyes(); break;
    case SQUINT: drawSquintEyes(); break;
    case HEART: drawHeartEyes(); break;
    case SLEEPY: drawSleepyEyes(); break;
    case X_X: drawDeadEyes(); break;
    case LEFT: drawNormalEyes(-10); break;
    case RIGHT: drawNormalEyes(10); break;
    case CENTER:
    default: drawNormalEyes(0); break;
  }
  display.display();
}

void FacialExpression::drawNormalEyes(int offsetX) {
  int leftX = 38, rightX = 90, y = 28, r = 18;
  display.fillCircle(leftX + offsetX, y, r, WHITE);
  display.fillCircle(rightX + offsetX, y, r, WHITE);
}

void FacialExpression::drawSquintEyes() {
  int leftX = 38, rightX = 90, y = 28, r = 18, thickness = 13;
  for (int i = -thickness/2; i <= thickness/2; i++) {
    display.drawLine(leftX - r, y + i, leftX + r, y + i, WHITE);
    display.drawLine(rightX - r, y + i, rightX + r, y + i, WHITE);
  }
}

void FacialExpression::drawCuteEyes() {
  int leftX = 38, rightX = 90, y = 35, r = 18;
  float angle_offset = 25 * 3.14159 / 180;
  for (int dy = -r; dy <= 0; dy++) {
    float dx = sqrt(r * r - dy * dy);
    float cutL = tan(angle_offset) * (-dy);
    float cutR = tan(-angle_offset) * (-dy);
    display.drawLine(leftX - dx + cutL, y + dy, leftX + dx + cutL, y + dy, WHITE);
    display.drawLine(rightX - dx + cutR, y + dy, rightX + dx + cutR, y + dy, WHITE);
  }
}

void FacialExpression::drawAngryEyes() {
  int leftX = 38, rightX = 90, y = 32, r = 18;
  float angle_deg = 28;
  float slope_left = tan(angle_deg * 3.14159 / 180.0);
  float slope_right = -tan(angle_deg * 3.14159 / 180.0);

  for(int dy = -r; dy <= r; dy++) {
    for(int dx = -r; dx <= r; dx++) {
      if(dx*dx + dy*dy <= r*r) {
        if (dy >= slope_left * dx) {
          display.drawPixel(leftX + dx, y + dy, WHITE);
        }
      }
    }
  }

  for(int dy = -r; dy <= r; dy++) {
    for(int dx = -r; dx <= r; dx++) {
      if(dx*dx + dy*dy <= r*r) {
        if (dy >= slope_right * dx) {
          display.drawPixel(rightX + dx, y + dy, WHITE);
        }
      }
    }
  }
}

void FacialExpression::drawHeartEyes() {
  drawHeart(38, 30);
  drawHeart(90, 30);
}

void FacialExpression::drawHeart(int x, int y) {
  // 32x32 超大爱心图案
  const uint8_t heart[32][32] = {
    {0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0},
    {0,0,1,1,1,1,1,1,1,1,1,1,0,0,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0},
    {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0},
    {0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0},
    {0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0},
    {0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0},
    {0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0},
    {0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,1,1,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0},
    {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0}
  };

  for (int j = 0; j < 32; j++) {
    for (int i = 0; i < 32; i++) {
      if (heart[j][i]) {
        display.drawPixel(x - 16 + i, y - 16 + j, WHITE);
      }
    }
  }
}

void FacialExpression::drawSleepyEyes() {
  int leftX = 38, rightX = 90, y = 35, r = 18;
  int thickness = 5;
  for (int i = -thickness / 2; i <= thickness / 2; i++) {
    display.drawLine(leftX - r, y + i, leftX + r, y + i, WHITE);
    display.drawLine(rightX - r, y + i, rightX + r, y + i, WHITE);
  }
  display.setTextSize(1);
  display.setTextColor(WHITE);
  display.setCursor(105, 0); display.print("Z");
  display.setCursor(110, 8); display.print("Z");
  display.setCursor(115, 16); display.print("Z");
}

void FacialExpression::drawDeadEyes() {
  drawX(38, 28, 10);
  drawX(90, 28, 10);
}

void FacialExpression::drawX(int x, int y, int size) {
  for (int i = -2; i <= 2; i++) {
    display.drawLine(x - size, y - size + i, x + size, y + size + i, WHITE); 
    display.drawLine(x - size, y + size + i, x + size, y - size + i, WHITE); 
  }
}

void FacialExpression::cycleNormalEyes() {
  // 清屏
  display.clearDisplay();

  // 根据当前状态画眼睛
  if (currentNormalDirection == CENTER) {
    drawNormalEyes(10); // 右偏
    currentNormalDirection = RIGHT;
  } 
  else if (currentNormalDirection == RIGHT) {
    drawNormalEyes(-10); // 左偏
    currentNormalDirection = LEFT;
  } 
  else {
    drawNormalEyes(0); // 中间
    currentNormalDirection = CENTER;
  }

  // 更新屏幕
  display.display();
}

void FacialExpression::update() {
  if (!isNormalMode) return; // 不是normal模式就啥也不做

  unsigned long now = millis();
  if (now - lastUpdateTime > 1000) { // 每隔1秒切换一次
    cycleNormalEyes();
    lastUpdateTime = now;
  }
}