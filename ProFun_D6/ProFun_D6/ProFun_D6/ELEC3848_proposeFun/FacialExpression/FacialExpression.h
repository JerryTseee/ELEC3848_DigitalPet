#ifndef FACIALEXPRESSION_H
#define FACIALEXPRESSION_H

#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define SCREEN_ADDRESS 0x3C

enum EyeDirection {
  CENTER, LEFT, RIGHT,
  SQUINT, ANGRY, CUTE,
  HEART, SLEEPY, X_X
};

class FacialExpression {
  public:
    FacialExpression();
    void begin();
    void fact(String expression);
    void update();//xinzeng
  private:
    Adafruit_SSD1306 display;
    EyeDirection currentNormalDirection = CENTER; // 新增的变量
    unsigned long lastUpdateTime = 0; // 上一次更新时间
    bool isNormalMode = false;        // 是不是normal模式

    void drawEyes(EyeDirection dir);
    void drawNormalEyes(int offsetX = 0);
    void drawSquintEyes();
    void drawCuteEyes();
    void drawAngryEyes();
    void drawHeartEyes();
    void drawHeart(int x, int y);
    void drawSleepyEyes();
    void drawDeadEyes();
    void drawX(int x, int y, int size);
    void cycleNormalEyes(); // 新增的方法
    
};

#endif