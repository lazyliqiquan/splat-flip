/* 项目思路

*/ 

//libs
#include <Arduino.h>
#include <Wire.h>
#include <Stepper.h>

//constants i2c
// 这一个翻牌模块单元的i2c地址，作为从机使用,第一个模块，后面的依次加一
#define i2cAddress 8  //i2c address

//constants stepper
// 控制步进电机的4个引脚
#define STEPPERPIN1 PA0
#define STEPPERPIN2 PA1
#define STEPPERPIN3 PA2
#define STEPPERPIN4 PA3
// 从起始位转弯一圈，回到起始位，需要多少个脉冲信号
#define STEPS 2038 // 28BYJ-48, number of steps;
// 2038 + 10，10 校准脉冲信号
#define CALOFFSET 10 //needs to be calibrated for each unit
// 霍尔传感器的pin引脚
#define HALLPIN PA4
// 显示文本的数量
#define AMOUNTFLAPS 45
// 步进电机的转动速度，单位：每分钟转多少圈
#define STEPPERSPEED 15 //in rpm
//thermistor
#define THERMISTORPIN A0
float thermistorB = 3950;
#define CRITICALTEMPERATURE 50 
bool stepperOverheated = false;
//constants others
// 波特率
#define BAUDRATE 9600
// 控制步进电机转动的方向
#define ROTATIONDIRECTION 1 //-1 for reverse direction
// 步进电机过热，间隔2秒后再移动
#define OVERHEATINGTIMEOUT 2 //timeout in seconds to avoid overheating of stepper. After starting rotation, the counter will start. Stepper won't move again until timeout is passed
// 字面翻译是：该模块上一次转动的时间
unsigned long lastRotation = 0;

//globals
// 显示的字母
String displayedLetter = " ";
// 想要显示的字母
String desiredLetter = " ";
const String letters[] = {" ","A","B","C","D","E","F","G","H","I","J","K","L","M","N","O","P","Q","R","S","T","U","V","W","X","Y","Z","Ä","Ö","Ü","0","1","2","3","4","5","6","7","8","9",":",".","-","?","!"};
Stepper stepper(STEPS, STEPPERPIN1,STEPPERPIN3,STEPPERPIN2,STEPPERPIN4); //stepper setup
// 存储阶段的最后状态
bool lastInd1 = false; //store last status of phase
bool lastInd2 = false; //store last status of phase
bool lastInd3 = false; //store last status of phase
bool lastInd4 = false; //store last status of phase
// 累积<1的步骤，当达到>1时通过额外的步骤进行补偿，所以它是一个浮点数
// 也就是累计误差吧，当误差大于一个脉冲的时候，就多发送一个脉冲，免得偏移误差过大
float missedSteps = 0; //cummulate steps <1, to compensate via additional step when reaching >1
// 缓存I2C读数据，三个字节是Unicode数据、一个是步进电机数据
byte byteBufferI2C[4]; //buffer for I2C read. Should be max. 4 bytes (three bytes for unicode and one for stop byte)
int amountBytesI2C = 0; //amount of bytes to convert
bool readBytesFinished = false; //if set to true, bytes will be converted to unicode letter inside loop

//function decleration
int calibrate();
void rotateToLetter(String toLetter);
float getTemperature();
void stopMotor();
void startMotor();
void receiveLetter(int amount);
void requestEvent(void);

//setup
void setup() {
  //initialize serial
  Serial.begin(BAUDRATE);

  //initialize i2c
  Serial.println("starting i2c slave");
  Wire.begin(i2cAddress); //i2c address of this unit 
  //注册接收数据的回调函数
  Wire.onReceive(receiveLetter); //call-function if for transfered letter via i2c 
  Wire.onRequest(requestEvent);

  //setup motor
  pinMode(HALLPIN, INPUT);
  calibrate(); //going to zero point
}

//loop
void loop() {
  //check if new bytes via I2C available for convert
  // 主机传送任务过来了，并且读取任务操作完成
  if(readBytesFinished) {
    //convert to utf string
    String utfLetter;
    //go through the bytes and get the utf letter
    for(int i = 0; i < amountBytesI2C; i++) {
      int charLength = 2; //at least two bytes including null termination
      if(bitRead(byteBufferI2C[i],7)&&bitRead(byteBufferI2C[i],6)) { //if true, utf char consists of at least two bytes + null termination
        charLength++;
        if(bitRead(byteBufferI2C[i],5)) charLength++; //if true, utf char consists of at least three bytes + null termination
        if(bitRead(byteBufferI2C[i],4)) charLength++; //if true, utf char consists of four bytes + null termination
      }
      byte aBuffer[charLength];
      for(int j = 0; j < charLength; j++) {
        aBuffer[j] = byteBufferI2C[i+j];
      }
      utfLetter = String((char*)aBuffer);
      i = i + (charLength-1); //skip bytes corresponding to length of utf letter
    }
    //convert small 'umlaute' to capital
    if(utfLetter == "ä") utfLetter = "Ä";
    if(utfLetter == "ö") utfLetter = "Ö";
    if(utfLetter == "ü") utfLetter = "Ü";
    desiredLetter = utfLetter;

    //reset values
    // 重置读取数据成功标志位
    readBytesFinished = false;
    amountBytesI2C = 0;
    byteBufferI2C[0] = 0;
    byteBufferI2C[1] = 0;
    byteBufferI2C[2] = 0;
    byteBufferI2C[3] = 0;
    Serial.println("1 out");
  }
  //temp for test calibration settings
  /*
  String calLetters[10] = {" ","Z","A","U","N","!","0","1","2","9"};
  for(int i = 0; i < 10; i++) {
    String currentCalLetter = calLetters[i];
    rotateToLetter(currentCalLetter);
    Serial.print("Current motor temperature:");
    Serial.print(getTemperature());
    Serial.println(" °C");
    delay(5000);
  }
  */
  //end temp

  //check for overheated motor
  if(getTemperature() < CRITICALTEMPERATURE) {
    //temperature ok
    stepperOverheated = false;
    //check if currently displayed letter differs from desired letter  
    // 如果目标字母和当前模块显示字母不一致，才有必要旋转
    if(displayedLetter!=desiredLetter) rotateToLetter(desiredLetter);
    delay(100);
  }
  else {
    //overheating alarm
    stopMotor();
    stepperOverheated = true;
    Serial.print("ciritcal temperature reached. current temperature:");
    Serial.print(getTemperature());
    Serial.println(" °C. turn off motor.");
    delay(10000);
  }
}

//doing a calibration of the revolver using the hall sensor
// 通过霍尔传感器对转盘进行校准
int calibrate() {
  Serial.println("calibrate revolver");
  // 是否到达标记点
  bool reachedMarker = false;
  // 设置步进电机的转速
  stepper.setSpeed(STEPPERSPEED);
  // i表示 校准整个校准过程中走了多少步
  int i = 0;
  while(!reachedMarker) {
    // 存储当前霍尔传感器的数值
    int currentHallValue = digitalRead(HALLPIN);
    // 已经处于零位，稍微移开一点再进行校准
    if(currentHallValue == 1 && i == 0) { //already in zero position move out a bit and do the calibration {
      //not reached yet
      i = 50;
      // 移动 50 步以脱离霍尔传感器的检测范围
      stepper.step(ROTATIONDIRECTION * 50); //move 50 steps to get out of scope of hall
      }
    else if(currentHallValue == 1) {
      //not reached yet
      // 尚未到达。
      stepper.step(ROTATIONDIRECTION * 1);
    }
    else {
      //reached marker, go to calibrated offset position
      reachedMarker = true;
      // 到达标记点，再移动10步就校准了
      stepper.step(ROTATIONDIRECTION * CALOFFSET);
      // 校准后显示的字母是空牌
      displayedLetter = " ";
      // 误差步数归零
      missedSteps = 0;
      stopMotor();
      return i;
    }
    // 转了三圈还是无法校准，停止继续校准，免得持续转动导致的电机过热，返回-1
    if(i > 3 * STEPS) {
      //seems that there is a problem with the marker or the sensor. turn of the motor to avoid overheating.
      displayedLetter = " ";
      desiredLetter = " ";
      reachedMarker = true;
      Serial.println("calibration revolver failed");
      return -1;
    }
    i++;
  }
  return i;
}

//rotate to desired letter
void rotateToLetter(String toLetter) {
  if(lastRotation == 0 || (millis() - lastRotation > OVERHEATINGTIMEOUT * 1000)) {
    lastRotation = millis();
    //get letter position
    // 目标字母在数组中的位置
    int posLetter = -1;
    // 当前字母在数组中的位置
    int posCurrentLetter = -1;
    int amountLetters = sizeof(letters) / sizeof(String);
    for(int i = 0; i < amountLetters; i++) {
      //current char
      String currentSearchLetter = letters[i];
      if(toLetter == currentSearchLetter) posLetter = i;
      if(displayedLetter == currentSearchLetter) posCurrentLetter = i;
    }
    Serial.print("go to letter:");
    Serial.println(toLetter);
    //go to letter, but only if available (>-1)
    // 判断想要显示的字母是否存在
    if(posLetter > -1) { //check if letter exists
      //check if letter is on higher index, then no full rotaion is needed
      // 判断想要转动到字母是在当前字母之前，还是之后
      if(posLetter >= posCurrentLetter) {
        Serial.println("direct");
        //go directly to next letter, get steps from current letter to target letter
        // 两者间隔的字母数
        int diffPosition = posLetter - posCurrentLetter;
        startMotor();
        stepper.setSpeed(STEPPERSPEED);
        //doing the rotation letterwise
        // 逐个字母，按顺序地转动
        for(int i=0;i<diffPosition;i++) {
          float preciseStep = (float)STEPS/(float)AMOUNTFLAPS;
          int roundedStep = (int)preciseStep;
          // 计算每转动一个字母生成的偏差，然后合计，看看是否大于一个脉冲，大于的话就有必要多转一圈了
          missedSteps = missedSteps + ((float)preciseStep - (float)roundedStep);
          if(missedSteps > 1) {
            roundedStep = roundedStep +1;
            missedSteps--;
          }
          stepper.step(ROTATIONDIRECTION*roundedStep);
        }
      }
      else {
        //full rotation is needed, good time for a calibration
        Serial.println("full rotation incl. calibration");
        // 表明加上这次要显示的字母，已经转了完整的一圈了，有必要通过霍尔传感器来定位旋转，而不是计算偏差，保证精度
        calibrate();
        startMotor();
        stepper.setSpeed(STEPPERSPEED);
        // 校准后，当前在原点，所以i从0开始计算
        for(int i=0;i<posLetter;i++) {
          float preciseStep = (float)STEPS/(float)AMOUNTFLAPS;
          int roundedStep = (int)preciseStep;
          missedSteps = missedSteps + (float)preciseStep - (float)roundedStep;
          if(missedSteps > 1) {
            roundedStep = roundedStep +1;
            missedSteps--;
          }
          stepper.step(ROTATIONDIRECTION*roundedStep);
        }
      }
      //store new position
      displayedLetter = toLetter;
      //rotation is done, stop the motor
      // 可能步进电机还没有旋转完整个流程，需要给点时间继续旋转，避免过早停止电机，导致旋转不到位
      delay(100); //important to stop rotation before shutting of the motor to avoid rotation after switching off current
      stopMotor();
    }
    else {
      Serial.println("letter unknown, go to space");
      desiredLetter = " ";
    }
  }
}

//temperature of motor
float getTemperature() {
  float thermistorReading = analogRead(THERMISTORPIN);
  float thermistorResistance = 100000/((1023/thermistorReading)-1);
  //Serial.println(thermistorResistance);
  float thermistorTemperature = (1/((log(thermistorResistance/100000.0)/thermistorB)+1/(273.15+25.0)))-273.15;
  return thermistorTemperature;
}

//switching off the motor driver
// 关闭电机驱动器
void stopMotor() {
  // 存储步进电机停止前的各个驱动引脚状态
  lastInd1 = digitalRead(STEPPERPIN1);
  lastInd2 = digitalRead(STEPPERPIN2);
  lastInd3 = digitalRead(STEPPERPIN3);
  lastInd4 = digitalRead(STEPPERPIN4);
  
  digitalWrite(STEPPERPIN1,LOW);
  digitalWrite(STEPPERPIN2,LOW);
  digitalWrite(STEPPERPIN3,LOW);
  digitalWrite(STEPPERPIN4,LOW);
}

void startMotor() {
  digitalWrite(STEPPERPIN1,lastInd1);
  digitalWrite(STEPPERPIN2,lastInd2);
  digitalWrite(STEPPERPIN3,lastInd3);
  digitalWrite(STEPPERPIN4,lastInd4);
}

void receiveLetter(int amount) {
  // 参数表示接收到的字节数
  amountBytesI2C = amount;
  int i = 0;
  while(Wire.available()){
    byteBufferI2C[i] = Wire.read();
    Serial.println(byteBufferI2C[i]);
    i++;
  }
  // 完成读取主机发送过来的数据
  readBytesFinished = true;
}

void requestEvent(void) {
  if(stepperOverheated) {
    Wire.write("O"); //sending tag o to master for overheating
    Serial.println("send O");
  }
  else {
    Wire.write("P"); //sending tag p to master for ping/alive
    Serial.println("send P");
  }
}
