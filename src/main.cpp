#include <Arduino.h>

#include <Arduino_FreeRTOS.h>
#include "pid.h"

pid_calibration PID_data = {
    .kp = 0, // Proportional gain
    .ki = 0, // Integral gain
    .kd = 0  // Derivative gain
};

pid_state PID_status = {
    .actual = 0,         // 测量的实际值
    .target = 0,         // 期望的值
    .time_delta = 0,     // 更新状态时应设置自上次采样/计算以来的时间
    .previous_error = 0, // 先前计算的实际与目标之间的误差(初始为零)
    .integral = 0,       // 积分误差随时间的总和
    .output = 0          // 修正后的输出值由算法计算，对误差进行补偿
};

// 定义电机控制引脚
const int L1_IN1 = 2;
const int L1_IN2 = 3; // 左上电机
const int L1_encoderPin = 18;
const int R1_IN1 = 4;
const int R1_IN2 = 5; // 右上电机
const int R1_encoderPin = 19;
const int L2_IN1 = 6;
const int L2_IN2 = 7; // 左下电机
const int L2_encoderPin = 20;
const int R2_IN1 = 8;
const int R2_IN2 = 9; // 右下电机
const int R2_encoderPin = 21;

// 各电机的占空比
const short ps_Min = 0;
const short ps_Max = 255;
double Car_ps = 100;
double L1_ps = Car_ps;
double R1_ps = Car_ps;
double L2_ps = Car_ps;
double R2_ps = Car_ps;

const short L1_Max = 7000;
const short R1_Max = 7280;
const short L2_Max = 7015;
const short R2_Max = 7100;

volatile double L1_encoderPos = 0;
volatile double L1_last_encoderPos = 0;
volatile double R1_encoderPos = 0;
volatile double R1_last_encoderPos = 0;
volatile double L2_encoderPos = 0;
volatile double L2_last_encoderPos = 0;
volatile double R2_encoderPos = 0;
volatile double R2_last_encoderPos = 0;

// 四个电机的平均速度
double middle_speed = 0;

// 当前时间，上一次时间，存储速度
unsigned long L1_now_Time = 0;
unsigned long L1_last_time = 0;
double L1_speed = 0;
unsigned long R1_now_Time = 0;
unsigned long R1_last_time = 0;
double R1_speed = 0;
unsigned long L2_now_Time = 0;
unsigned long L2_last_time = 0;
double L2_speed = 0;
unsigned long R2_now_Time = 0;
unsigned long R2_last_time = 0;
double R2_speed = 0;

// 电机运动状态
bool L1_forward_Status = false;
bool L1_backward_Status = false;
bool R1_forward_Status = false;
bool R1_backward_Status = false;
bool L2_forward_Status = false;
bool L2_backward_Status = false;
bool R2_forward_Status = false;
bool R2_backward_Status = false;
bool allstop_Status = false;

double PID_output(short encoderPos_Max, double middle_encoderPos, double new_encoderPos)
{
  // PID_data.kp = 255.0 / encoderPos_Max;
  // PID_data.ki = 0;
  // PID_data.kd = 0;

  // PID_status.actual = new_encoderPos;
  // PID_status.target = middle_encoderPos;
  // PID_status.time_delta = 0.3; 

  // pid_iterate(PID_data, PID_status);
  // return PID_status.output;

  double PID_out = 255.0 / encoderPos_Max * (middle_encoderPos - new_encoderPos) + 0.3 * (middle_encoderPos - new_encoderPos);

  return PID_out;
}

double Judge_data(double data)
{
  if (data >= ps_Max)
  {
    data = ps_Max;
  }
  else if (data <= ps_Min)
  {
    data = ps_Min;
  }
  return data;
}

// 比较函数，用于在 qsort 中比较两个元素
int compare(const void *a, const void *b)
{
  return (*(int *)a - *(int *)b);
}

void PIDTask(void *pvParameters)
{
  while (1)
  {
    double speeds[] = {L1_speed, R1_speed, L2_speed, R2_speed};
    double size = sizeof(speeds) / sizeof(speeds[0]);

    // 使用 qsort 函数进行升序排序
    qsort(speeds, size, sizeof(speeds[0]), compare);

    // 中间速度是排序后数组的第三个元素
    middle_speed = speeds[2];

    L1_ps = L1_ps + PID_output(L1_Max, middle_speed, L1_speed);
    R1_ps = R1_ps + PID_output(R1_Max, middle_speed, R1_speed);
    L2_ps = L2_ps + PID_output(L2_Max, middle_speed, L2_speed);
    R2_ps = R2_ps + PID_output(R2_Max, middle_speed, R2_speed);

    L1_ps = Judge_data(L1_ps);
    R1_ps = Judge_data(R1_ps);
    L2_ps = Judge_data(L2_ps);
    R2_ps = Judge_data(R2_ps);

    Serial.print(L1_ps);
    Serial.print(" ");
    Serial.print(R1_ps);
    Serial.print(" ");
    Serial.print(L2_ps);
    Serial.print(" ");
    Serial.println(R2_ps);

    Serial.print(L1_speed);
    Serial.print(" ");
    Serial.print(R1_speed);
    Serial.print(" ");
    Serial.print(L2_speed);
    Serial.print(" ");
    Serial.println(R2_speed);

    vTaskDelay(pdMS_TO_TICKS(300));
  }
}

// void testTask(void *pvParameters) {
//     while (1)
//     {

//       // Serial.print(L1_speed);
//       // Serial.print(" ");
//       // Serial.print(R1_speed);
//       // Serial.print(" ");
//       // Serial.print(L2_speed);
//       // Serial.print(" ");
//       // Serial.println(R2_speed);

//       // Serial.print(L1_ps);
//       // Serial.print(" ");
//       // Serial.print(R1_ps);
//       // Serial.print(" ");
//       // Serial.print(L2_ps);
//       // Serial.print(" ");
//       // Serial.println(R2_ps);
//       vTaskDelay(pdMS_TO_TICKS(50));
//     }
// }

void motor_Task(void *pvParameters)
{
  while (1)
  {
    if (L1_forward_Status)
    {
      // 左前轮前进
      analogWrite(L1_IN1, L1_ps);
      digitalWrite(L1_IN2, LOW);
    }

    if (L1_backward_Status)
    {
      // 左前轮后退
      digitalWrite(L1_IN1, LOW);
      analogWrite(L1_IN2, L1_ps);
    }

    if (R1_forward_Status)
    {
      // 右前轮前进
      analogWrite(R1_IN1, R1_ps);
      digitalWrite(R1_IN2, LOW);
    }

    if (R1_backward_Status)
    {
      digitalWrite(R1_IN1, LOW);
      analogWrite(R1_IN2, R1_ps);
    }

    if (L2_forward_Status)
    { // 左后轮前进
      analogWrite(L2_IN1, L2_ps);
      digitalWrite(L2_IN2, LOW);
    }

    if (L2_backward_Status)
    {
      // 左后轮后退
      digitalWrite(L2_IN1, LOW);
      analogWrite(L2_IN2, L2_ps);
    }

    if (R2_forward_Status)
    { // 右后轮前进
      analogWrite(R2_IN1, R2_ps);
      digitalWrite(R2_IN2, LOW);
    }

    if (R2_backward_Status)
    { // 右后轮后退
      digitalWrite(R2_IN1, LOW);
      analogWrite(R2_IN2, R2_ps);
    }

    if (allstop_Status)
    {
      digitalWrite(L1_IN1, LOW);
      digitalWrite(L1_IN2, LOW);
      digitalWrite(R1_IN1, LOW);
      digitalWrite(R1_IN2, LOW);
      digitalWrite(L2_IN1, LOW);
      digitalWrite(L2_IN2, LOW);
      digitalWrite(R2_IN1, LOW);
      digitalWrite(R2_IN2, LOW);

      L1_forward_Status = false;
      L1_backward_Status = false;
      R1_forward_Status = false;
      R1_backward_Status = false;
      L2_forward_Status = false;
      L2_backward_Status = false;
      R2_forward_Status = false;
      R2_backward_Status = false;
    }

    vTaskDelay(pdMS_TO_TICKS(200));
  }
}

void moveTask(void *pvParameters)
{
  while (1)
  {
    //  if(Serial.available()){
    //     Serial_data = Serial.read();
    //  }

    Serial.println("前进");
    /*前进*/
    allstop_Status = false;
    L1_forward_Status = true;
    R1_forward_Status = true;
    L2_forward_Status = true;
    R2_forward_Status = true;
    vTaskDelay(pdMS_TO_TICKS(5000)); // 挂起任务3秒
    allstop_Status = true;
    vTaskDelay(pdMS_TO_TICKS(1000));

    Serial.println("后退");
    /*后退*/
    allstop_Status = false;
    L1_backward_Status = true;
    R1_backward_Status = true;
    L2_backward_Status = true;
    R2_backward_Status = true;
    vTaskDelay(pdMS_TO_TICKS(5000)); // 挂起任务3秒
    allstop_Status = true;
    vTaskDelay(pdMS_TO_TICKS(1000));

    // Serial.println("顺时针原地旋转");
    // /*顺时针原地旋转*/
    // L1_forward(L1_ps);
    // R1_backward(R1_ps);
    // L2_forward(L2_ps);
    // R2_backward(R2_ps);
    // vTaskDelay(pdMS_TO_TICKS(3000));  // 挂起任务3秒
    // allstop();
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // Serial.println("逆时针原地旋转");
    // /*逆时针原地旋转*/
    // L1_backward(L1_ps);
    // R1_forward(R1_ps);
    // L2_backward(L2_ps);
    // R2_forward(R2_ps);
    // vTaskDelay(pdMS_TO_TICKS(3000));  // 挂起任务3秒
    // allstop();
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // Serial.println("左边平移");
    // /*左边平移*/
    // L1_backward(L1_ps);
    // R1_forward(R1_ps);
    // L2_forward(L2_ps);
    // R2_backward(R2_ps);
    // vTaskDelay(pdMS_TO_TICKS(3000));  // 挂起任务3秒
    // allstop();
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // Serial.println("右边平移");
    // /*右边平移*/
    // L1_forward(L1_ps);
    // R1_backward(R1_ps);
    // L2_backward(L2_ps);
    // R2_forward(R2_ps);
    // vTaskDelay(pdMS_TO_TICKS(3000));  // 挂起任务3秒
    // allstop();
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // Serial.println("斜向左上方");
    // /*斜向左上方*/
    // R1_forward(R1_ps);
    // L2_forward(L2_ps);
    // vTaskDelay(pdMS_TO_TICKS(3000));  // 挂起任务3秒
    // allstop();
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // Serial.println("斜向右上方");
    // /*斜向右上方*/
    // L1_forward(L1_ps);
    // R2_forward(R2_ps);
    // vTaskDelay(pdMS_TO_TICKS(3000));  // 挂起任务3秒
    // allstop();
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // Serial.println("斜向左下方");
    // /*斜向左下方*/
    // L1_backward(L1_ps);
    // R2_backward(R2_ps);
    // vTaskDelay(pdMS_TO_TICKS(3000));  // 挂起任务3秒
    // allstop();
    // vTaskDelay(pdMS_TO_TICKS(1000));

    // Serial.println("斜向右下方");
    // /*斜向右下方*/
    // R1_backward(R1_ps);
    // L2_backward(L2_ps);
    // vTaskDelay(pdMS_TO_TICKS(3000));  // 挂起任务3秒
    // allstop();
    // vTaskDelay(pdMS_TO_TICKS(1000));
  }
}

void L1_doEncoder()
{
  // 计算速度
  L1_now_Time = millis();
  if (L1_now_Time - L1_last_time >= 500) // 每秒计算一次速度
  {
    L1_speed = (float)(L1_encoderPos - L1_last_encoderPos) / (float)(L1_now_Time - L1_last_time) * 1000.0;
    // Serial.print("Speed: ");
    // Serial.println(L1_speed);

    L1_last_encoderPos = L1_encoderPos;
    L1_last_time = L1_now_Time;
  }

  // L1_last_encoderPos = L1_encoderPos;
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
  if (R1_now_Time - R1_last_time >= 500) // 每秒计算一次速度
  {
    R1_speed = (float)(R1_encoderPos - R1_last_encoderPos) / (float)(R1_now_Time - R1_last_time) * 1000.0;
    // Serial.print("Speed: ");
    // Serial.println(R1_speed);

    R1_last_encoderPos = R1_encoderPos;
    R1_last_time = R1_now_Time;
  }

  // L1_last_encoderPos = L1_encoderPos;
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
  if (L2_now_Time - L2_last_time >= 500) // 每秒计算一次速度
  {
    L2_speed = (float)(L2_encoderPos - L2_last_encoderPos) / (float)(L2_now_Time - L2_last_time) * 1000.0;
    // Serial.print("Speed: ");
    // Serial.println(L2_speed);

    L2_last_encoderPos = L2_encoderPos;
    L2_last_time = L2_now_Time;
  }

  // L1_last_encoderPos = L1_encoderPos;
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
  if (R2_now_Time - R2_last_time >= 500) // 每秒计算一次速度
  {
    R2_speed = (float)(R2_encoderPos - R2_last_encoderPos) / (float)(R2_now_Time - R2_last_time) * 1000.0;
    // Serial.print("Speed: ");
    // Serial.println(R2_speed);

    R2_last_encoderPos = R2_encoderPos;
    R2_last_time = R2_now_Time;
  }

  // L1_last_encoderPos = L1_encoderPos;
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
  xTaskCreate(PIDTask, "通过PID+编码器检测速度", 1000, NULL, 2, NULL);
  // xTaskCreate(testTask, "通过编码器检测速度", 1000, NULL, 3, NULL);
  xTaskCreate(moveTask, "电机动作", 1000, NULL, 1, NULL);
  xTaskCreate(motor_Task, "改变电机状态", 1000, NULL, 3, NULL);

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