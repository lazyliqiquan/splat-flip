/* 项目思路，分成两个模块，一个是unit，一个是master
master：带有esp8266通信模块，负责外界通信，控制整个项目和控制自己所属的一个翻转模块
unit：控制自己所属的翻转模块，同时作为从机接收master的指令

显示文本：0-9、A-Z、[空 ❤ + - / ？ ！ ： @] 一共45个字符

运行模式有4种(5个显示位)：
日期：MM-DD
时间：24小时格式， HH:MM
文本：显示指定的文本
算数：(如果位数不足以占满五个显示位，就往右边靠紧)，接收到一次指令，就由master控制安排5局games，
一局完整的game：随机对局->展示题目(过程消耗2s)->间隔7s->展示答案(过程消耗2s)->10s->new game
加、减(结果可以存在负数)、乘、除(保证 被除数 能整除 除数)

TODO:关于步进电机持续转动过热问题，其实只有两种情况会出现过热，无非是 频繁显示文本 和 各个运行模式间频繁切换
总结就是控制端频繁发送指令，解决办法：简单暴力，直接在控制端遏制发送的频率，比如每两次指令间的间隔至少为10s，
这样master只要接收到指令，就立马去落实(不怕过热，因为控制端已经间隔指令了)，即使当前还有指令未执行完毕
*/

// libs
#include <Arduino.h>
#include <Wire.h>
#include <Stepper.h>

// constants i2c
//  这一个翻牌模块单元的i2c地址，作为从机使用,第一个模块，后面的依次加一
#define i2cAddress 8 // i2c address

// constants stepper
//  控制步进电机的4个引脚
#define STEPPERPIN1 PA0
#define STEPPERPIN2 PA1
#define STEPPERPIN3 PA2
#define STEPPERPIN4 PA3
// 从起始位转弯一圈，回到起始位，需要多少个脉冲信号
#define STEPS 2038 // 28BYJ-48, number of steps;
// 2038 + 10，10 校准脉冲信号
#define CALOFFSET 10 // needs to be calibrated for each unit
// 霍尔传感器的pin引脚
#define HALLPIN PA4
// 显示文本的数量
#define AMOUNTFLAPS 45
// 步进电机的转动速度，单位：每分钟转多少圈
#define STEPPERSPEED 15 // in rpm

// constants others
//  波特率
#define BAUDRATE 9600
// 控制步进电机转动的方向
#define ROTATIONDIRECTION 1 //-1 for reverse direction
// FIXME: 步进电机过热，间隔2秒后再移动
#define OVERHEATINGTIMEOUT 2 // timeout in seconds to avoid overheating of stepper. After starting rotation, the counter will start. Stepper won't move again until timeout is passed
// 字面翻译是：该模块上一次转动的时间
unsigned long lastRotation = 0;

// globals
//  显示的字母
int8_t displayedLetter = 0;
// 想要显示的字母
int8_t desiredLetter = 0;
// 可显示字符列表
const String letters[] = {" ", "A", "B", "C", "D", "E", "F", "G", "H", "I", "J", "K", "L", "M", "N", "O", "P", "Q", "R", "S", "T", "U", "V", "W", "X", "Y", "Z", "0", "1", "2", "3", "4", "5", "6", "7", "8", "9", "*", ":", "+", "-", "?", "!", "@", "/"};
Stepper stepper(STEPS, STEPPERPIN1, STEPPERPIN3, STEPPERPIN2, STEPPERPIN4); // stepper setup
// 存储阶段的最后状态
bool lastInd1 = false; // store last status of phase
bool lastInd2 = false; // store last status of phase
bool lastInd3 = false; // store last status of phase
bool lastInd4 = false; // store last status of phase
// 累积<1的步骤，当达到>1时通过额外的步骤进行补偿，所以它是一个浮点数
// 也就是累计误差吧，当误差大于一个脉冲的时候，就多发送一个脉冲，免得偏移误差过大
float missedSteps = 0; // cummulate steps <1, to compensate via additional step when reaching >1
int8_t byteFromI2C = -1;
bool readBytesFinished = false; // if set to true, bytes will be converted to unicode letter inside loop

// function decleration
int calibrate();
void rotateToLetter(int8_t toLetter);
void stopMotor();
void startMotor();
void receiveLetter(int amount);
void requestEvent(void);

// setup
void setup()
{
  // initialize serial
  Serial.begin(BAUDRATE);
  // debug完可以注释掉下面的延迟函数
  delay(500);
  // initialize i2c
  Serial.println("starting i2c slave");
  Wire.begin(i2cAddress); // i2c address of this unit
  // 注册接收数据的回调函数
  Wire.onReceive(receiveLetter); // call-function if for transfered letter via i2c
  // 可以发送当前存在的问题情况给master
  Wire.onRequest(requestEvent);
  // setup motor
  pinMode(HALLPIN, INPUT);
  calibrate(); // going to zero point
}

void loop()
{
  // check if new bytes via I2C available for convert
  //  主机传送任务过来了，并且读取任务操作完成
  if (readBytesFinished)
  {
    desiredLetter = byteFromI2C;
    // reset values
    //  重置读取数据成功标志位
    readBytesFinished = false;
    byteFromI2C = -1;
    Serial.println("1 out");
  }
  // debug：注意， 为了防止步进电机过热，命令还是有必要控制一下频率
  // check if currently displayed letter differs from desired letter
  // 如果目标字母和当前模块显示字母不一致，才有必要旋转
  if (displayedLetter != desiredLetter)
    rotateToLetter(desiredLetter);
  delay(100);
}

// doing a calibration of the revolver using the hall sensor
//  通过霍尔传感器对转盘进行校准
int calibrate()
{
  Serial.println("calibrate revolver");
  // 是否到达标记点
  bool reachedMarker = false;
  // 设置步进电机的转速
  stepper.setSpeed(STEPPERSPEED);
  // i表示 校准整个校准过程中走了多少步
  int i = 0;
  while (!reachedMarker)
  {
    // 存储当前霍尔传感器的数值,获取数字信号，只有0和1
    // 
    int currentHallValue = digitalRead(HALLPIN);
    // 已经处于零位，稍微移开一点再进行校准
    if (currentHallValue == 1 && i == 0)
    { // already in zero position move out a bit and do the calibration {
      // not reached yet
      i = 50;
      // 移动 50 步以脱离霍尔传感器的检测范围
      stepper.step(ROTATIONDIRECTION * 50); // move 50 steps to get out of scope of hall
    }
    else if (currentHallValue == 1)
    {
      // not reached yet
      //  尚未到达。
      stepper.step(ROTATIONDIRECTION * 1);
    }
    else
    {
      // reached marker, go to calibrated offset position
      reachedMarker = true;
      // 到达标记点，再移动10步就校准了
      stepper.step(ROTATIONDIRECTION * CALOFFSET);
      // 校准后显示的字母是空牌
      displayedLetter = 0;
      // 误差步数归零
      missedSteps = 0;
      stopMotor();
      return i;
    }
    // 转了三圈还是无法校准，停止继续校准，免得持续转动导致的电机过热，返回-1
    if (i > 3 * STEPS)
    {
      // seems that there is a problem with the marker or the sensor. turn of the motor to avoid overheating.
      displayedLetter = 0;
      desiredLetter = 0;
      reachedMarker = true;
      Serial.println("calibration revolver failed");
      return -1;
    }
    i++;
  }
  return i;
}

// rotate to desired letter
void rotateToLetter(int8_t toLetter)
{
  // 如果是第一次旋转 或者 本次旋转距离上一次旋转间隔时间超过2s 才可以执行本次操作
  if (lastRotation == 0 || (millis() - lastRotation > OVERHEATINGTIMEOUT * 1000))
  {
    lastRotation = millis();
    // go to letter, but only if available (>-1)
    //  判断想要显示的字母是否存在
    if (toLetter > -1 && toLetter < AMOUNTFLAPS)
    { // check if letter exists
      // check if letter is on higher index, then no full rotaion is needed
      //  判断想要转动到字母是在当前字母之前，还是之后
      if (toLetter >= displayedLetter)
      {
        // 如果想要移动到的字母在当前字母之后，那么直接往后移动一定的步数即可
        Serial.println("direct");
        // go directly to next letter, get steps from current letter to target letter
        //  两者间隔的字母数
        int8_t diffPosition = toLetter - displayedLetter;
        startMotor();
        stepper.setSpeed(STEPPERSPEED);
        // doing the rotation letterwise
        //  逐个字母，按顺序地转动
        for (int8_t i = 0; i < diffPosition; i++)
        {
          float preciseStep = (float)STEPS / (float)AMOUNTFLAPS;
          int roundedStep = (int)preciseStep;
          // 计算每转动一个字母生成的偏差，然后合计，看看是否大于一个脉冲，大于的话就有必要多转一圈了
          missedSteps = missedSteps + ((float)preciseStep - (float)roundedStep);
          if (missedSteps > 1)
          {
            roundedStep = roundedStep + 1;
            missedSteps--;
          }
          stepper.step(ROTATIONDIRECTION * roundedStep);
        }
      }
      else
      {
        // full rotation is needed, good time for a calibration
        Serial.println("full rotation incl. calibration");
        // 表明加上这次要显示的字母，已经转了完整的一圈了，有必要通过霍尔传感器来定位旋转，而不是计算偏差，保证精度
        calibrate();
        startMotor();
        stepper.setSpeed(STEPPERSPEED);
        // 校准后，当前在原点，所以i从0开始计算
        for (int i = 0; i < toLetter; i++)
        {
          float preciseStep = (float)STEPS / (float)AMOUNTFLAPS;
          int roundedStep = (int)preciseStep;
          missedSteps = missedSteps + (float)preciseStep - (float)roundedStep;
          if (missedSteps > 1)
          {
            roundedStep = roundedStep + 1;
            missedSteps--;
          }
          stepper.step(ROTATIONDIRECTION * roundedStep);
        }
      }
      // store new position
      displayedLetter = toLetter;
      // rotation is done, stop the motor
      //  可能步进电机还没有旋转完整个流程，需要给点时间继续旋转，避免过早停止电机，导致旋转不到位
      delay(100); // important to stop rotation before shutting of the motor to avoid rotation after switching off current
      stopMotor();
    }
    else
    {
      Serial.println("letter unknown, go to space");
      desiredLetter = 0;
    }
  }
}

// switching off the motor driver
//  关闭电机驱动器
void stopMotor()
{
  // 存储步进电机停止前的各个驱动引脚状态
  lastInd1 = digitalRead(STEPPERPIN1);
  lastInd2 = digitalRead(STEPPERPIN2);
  lastInd3 = digitalRead(STEPPERPIN3);
  lastInd4 = digitalRead(STEPPERPIN4);

  digitalWrite(STEPPERPIN1, LOW);
  digitalWrite(STEPPERPIN2, LOW);
  digitalWrite(STEPPERPIN3, LOW);
  digitalWrite(STEPPERPIN4, LOW);
}

void startMotor()
{
  digitalWrite(STEPPERPIN1, lastInd1);
  digitalWrite(STEPPERPIN2, lastInd2);
  digitalWrite(STEPPERPIN3, lastInd3);
  digitalWrite(STEPPERPIN4, lastInd4);
}

// 参数表示接收到的字节数
void receiveLetter(int amount)
{
  while (Wire.available())
  {
    byteFromI2C = Wire.read();
    Serial.println(byteFromI2C);
  }
  // 完成读取主机发送过来的数据
  readBytesFinished = true;
}

// 每个unit自己维护校准，不需要向master传输信息
void requestEvent(void)
{
}
