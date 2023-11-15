#include <Arduino.h>

#include <Arduino_FreeRTOS.h>
#include "pid.h"

struct pid_calibration PID_data = {
  .kp = 0.02, // Proportional gain
  .ki = 0.01, // Integral gain
  .kd = 0.01 // Derivative gain
};

struct pid_state PID_status = {
  .actual = 0.01, // 测量的实际值
  .target = 0.01, // 期望的值
  .time_delta = 0.01, // 更新状态时应设置自上次采样/计算以来的时间
  .previous_error = 0.01, //先前计算的实际与目标之间的误差(初始为零)
  .integral = 0.01, // 积分误差随时间的总和
  .output = 0.01 // 修正后的输出值由算法计算，对误差进行补偿
};


// 定义电机控制引脚
const int L1_IN1 = 2;const int L1_IN2 = 3; // 左上电机
const int L1_encoderPin = 18;
const int R1_IN1 = 4;const int R1_IN2 = 5; // 右上电机
const int R1_encoderPin = 19;
const int L2_IN1 = 6;const int L2_IN2 = 7; // 左下电机
const int L2_encoderPin = 20;
const int R2_IN1 = 8;const int R2_IN2 = 9; // 右下电机
const int R2_encoderPin = 21;

char Serial_data = ' ';

//各电机的占空比
int Car_ps = 100;
int L1_ps = Car_ps;
int R1_ps = Car_ps;
int L2_ps = Car_ps;
int R2_ps = Car_ps;

volatile long L1_encoderPos = 0;volatile long L1_last_encoderPos = 0;
volatile long R1_encoderPos = 0;volatile long R1_last_encoderPos = 0;
volatile long L2_encoderPos = 0;volatile long L2_last_encoderPos = 0;
volatile long R2_encoderPos = 0;volatile long R2_last_encoderPos = 0;

unsigned long L1_now_Time = 0;long L1_last_time = 0;float L1_speed = 0.0;// 当前时间。上一次时间，存储速度
unsigned long R1_now_Time = 0;long R1_last_time = 0;float R1_speed = 0.0;// 当前时间。上一次时间，存储速度
unsigned long L2_now_Time = 0;long L2_last_time = 0;float L2_speed = 0.0;// 当前时间。上一次时间，存储速度
unsigned long R2_now_Time = 0;long R2_last_time = 0;float R2_speed = 0.0;// 当前时间。上一次时间，存储速度

void L1_forward(int sp) // 左前轮前进
{
  analogWrite(L1_IN1, sp);
  digitalWrite(L1_IN2, LOW);
}
void R1_forward(int sp) // 右前轮前进
{
  analogWrite(R1_IN1, sp);
  digitalWrite(R1_IN2, LOW);

}
void L2_forward(int sp) // 左后轮前进
{
  analogWrite(L2_IN1, sp);
  digitalWrite(L2_IN2, LOW);
}
void R2_forward(int sp) // 右后轮前进
{
  analogWrite(R2_IN1, sp);
  digitalWrite(R2_IN2, LOW);
}
void allstop()
{
  digitalWrite(L1_IN1, LOW);
  digitalWrite(L1_IN2, LOW);
  digitalWrite(R1_IN1, LOW);
  digitalWrite(R1_IN2, LOW);
  digitalWrite(L2_IN1, LOW);
  digitalWrite(L2_IN2, LOW);
  digitalWrite(R2_IN1, LOW);
  digitalWrite(R2_IN2, LOW);
}
void L1_backward(int sp) // 左前轮后退
{
  digitalWrite(L1_IN1, LOW);
  analogWrite(L1_IN2, sp);
}
void R1_backward(int sp) // 右前轮后退
{
  digitalWrite(R1_IN1, LOW);
  analogWrite(R1_IN2, sp);
}
void L2_backward(int sp) // 左后轮后退
{
  digitalWrite(L2_IN1, LOW);
  analogWrite(L2_IN2, sp);
}
void R2_backward(int sp) // 右后轮后退
{
  digitalWrite(R2_IN1, LOW);
  analogWrite(R2_IN2, sp);
}

void moveTask(void *pvParameters) {
  while (1) {
    //  if(Serial.available()){
    //     Serial_data = Serial.read();
    //  }

    Serial.println("前进");
    /*前进*/
    L1_forward(L1_ps);
    R1_forward(R1_ps);
    L2_forward(L2_ps);
    R2_forward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("后退");
    /*后退*/
    L1_backward(L1_ps);
    R1_backward(R1_ps);
    L2_backward(L2_ps);
    R2_backward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("顺时针原地旋转");
    /*顺时针原地旋转*/
    L1_forward(L1_ps);
    R1_backward(R1_ps);
    L2_forward(L2_ps);
    R2_backward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("逆时针原地旋转");
    /*逆时针原地旋转*/
    L1_backward(L1_ps);
    R1_forward(R1_ps);
    L2_backward(L2_ps);
    R2_forward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("左边平移");
    /*左边平移*/
    L1_backward(L1_ps);
    R1_forward(R1_ps);
    L2_forward(L2_ps);
    R2_backward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("右边平移");
    /*右边平移*/
    L1_forward(L1_ps);
    R1_backward(R1_ps);
    L2_backward(L2_ps);
    R2_forward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("斜向左上方");
    /*斜向左上方*/
    R1_forward(R1_ps);
    L2_forward(L2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("斜向右上方");
    /*斜向右上方*/
    L1_forward(L1_ps);
    R2_forward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("斜向左下方");
    /*斜向左下方*/
    L1_backward(L1_ps);
    R2_backward(R2_ps);
    delay(5000);
    allstop();
    delay(1500);

    Serial.println("斜向右下方");
    /*斜向右下方*/
    R1_backward(R1_ps);
    L2_backward(L2_ps);
    delay(5000);
    allstop();
    delay(1500);
  }
}

void testTask(void *pvParameters) {
    while (1) 
    {    
        long middle_speed = (L1_speed + R1_speed + L2_speed + R2_speed)/4;
        if(L1_speed > middle_speed){
          L1_ps--;
        }else if(L1_speed < middle_speed){
          L1_ps++;
        }
        if(R1_speed > middle_speed){
          R1_ps--;
        }else if(R1_speed < middle_speed){
          R1_ps++;
        }
        if(L2_speed > middle_speed){
          L2_ps--;
        }else if(L2_speed < middle_speed){
          L2_ps++;
        }
        if(R2_speed > middle_speed){
          R2_ps--;
        }else if(R2_speed < middle_speed){
          R2_ps++;
        }
        
        delay(200);
    }
}

void L1_doEncoder()
{
  // 计算速度
  L1_now_Time = millis();
  if (L1_now_Time - L1_last_time >= 1000) // 每秒计算一次速度
  {
    L1_speed = (float)(L1_encoderPos - L1_last_encoderPos) / (float)(L1_now_Time - L1_last_time) * 1000.0;
    Serial.print("Speed: ");
    Serial.println(L1_speed);

    L1_last_encoderPos = L1_encoderPos;
    L1_last_time = L1_now_Time;
  }

  //L1_last_encoderPos = L1_encoderPos;
  if (digitalRead(L1_encoderPin) == HIGH)
  {
    L1_encoderPos += 2;
  }
  else if (digitalRead(L1_encoderPin) == LOW)
  {
    L1_encoderPos += 2;
  }
}

void R1_doEncoder()
{
  // 计算速度
  R1_now_Time = millis();
  if (R1_now_Time - R1_last_time >= 1000) // 每秒计算一次速度
  {
    R1_speed = (float)(R1_encoderPos - R1_last_encoderPos) / (float)(R1_now_Time - R1_last_time) * 1000.0;
    Serial.print("Speed: ");
    Serial.println(R1_speed);

    R1_last_encoderPos = R1_encoderPos;
    R1_last_time = R1_now_Time;
  }

  //L1_last_encoderPos = L1_encoderPos;
  if (digitalRead(R1_encoderPin) == HIGH)
  {
    R1_encoderPos += 2;
  }
  else if (digitalRead(R1_encoderPin) == LOW)
  {
    R1_encoderPos += 2;
  }
}
void L2_doEncoder()
{
  // 计算速度
  L2_now_Time = millis();
  if (L2_now_Time - L2_last_time >= 1000) // 每秒计算一次速度
  {
    L2_speed = (float)(L2_encoderPos - L2_last_encoderPos) / (float)(L2_now_Time - L2_last_time) * 1000.0;
    Serial.print("Speed: ");
    Serial.println(L2_speed);

    L2_last_encoderPos = L2_encoderPos;
    L2_last_time = L2_now_Time;
  }

  //L1_last_encoderPos = L1_encoderPos;
  if (digitalRead(L2_encoderPin) == HIGH)
  {
    L2_encoderPos += 2;
  }
  else if (digitalRead(L2_encoderPin) == LOW)
  {
    L2_encoderPos += 2;
  }
}

void R2_doEncoder()
{
  // 计算速度
  R2_now_Time = millis();
  if (R2_now_Time - R2_last_time >= 1000) // 每秒计算一次速度
  {
    R2_speed = (float)(R2_encoderPos - R2_last_encoderPos) / (float)(R2_now_Time - R2_last_time) * 1000.0;
    Serial.print("Speed: ");
    Serial.println(R2_speed);

    R2_last_encoderPos = R2_encoderPos;
    R2_last_time = R2_now_Time;
  }

  //L1_last_encoderPos = L1_encoderPos;
  if (digitalRead(R2_encoderPin) == HIGH)
  {
    R2_encoderPos += 2;
  }
  else if (digitalRead(R2_encoderPin) == LOW)
  {
    R2_encoderPos += 2;
  }
}




void setup()
{
  xTaskCreate(moveTask, "电机动作", 1000, NULL, 1, NULL); // 创建任务
  xTaskCreate(testTask, "通过编码器检测速度", 1000, NULL, 2, NULL); // 创建任务

  attachInterrupt(digitalPinToInterrupt(L1_encoderPin), L1_doEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R1_encoderPin), R1_doEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(L2_encoderPin), L2_doEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(R2_encoderPin), R2_doEncoder, CHANGE);

  Serial.begin(115200);

  pinMode(L1_encoderPin, INPUT);
  pinMode(R1_encoderPin, INPUT);
  pinMode(L2_encoderPin, INPUT);
  pinMode(R2_encoderPin, INPUT);


  pinMode(L1_IN1, OUTPUT);
  pinMode(L1_IN2, OUTPUT);
  pinMode(R1_IN1, OUTPUT);
  pinMode(R1_IN2, OUTPUT);
  pinMode(L2_IN1, OUTPUT);
  pinMode(L2_IN2, OUTPUT);
  pinMode(R2_IN1, OUTPUT);
  pinMode(R2_IN2, OUTPUT);

  Serial.println("初始化完毕....");
}




void loop()
{

}