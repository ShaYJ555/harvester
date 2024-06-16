#include <HardwareSerial.h>   // 硬件串口

#define MOTOR3_IN1  15
#define MOTOR3_IN2  2
#define MOTOR4_IN1  0
#define MOTOR4_IN2  4
#define MOTOR5_IN1  5
#define MOTOR5_IN2  18
#define MOTOR6_IN1  23
#define MOTOR6_IN2  19

#define MOTOR3_IN1_CHANNEL   0
#define MOTOR3_IN2_CHANNEL   1
#define MOTOR4_IN1_CHANNEL   2
#define MOTOR4_IN2_CHANNEL   3
#define MOTOR5_IN1_CHANNEL   4
#define MOTOR5_IN2_CHANNEL   5
#define MOTOR6_IN1_CHANNEL   6
#define MOTOR6_IN2_CHANNEL   7

#define PWM_FREQ        1000
#define PWM_RESOLUTION  8

HardwareSerial SerialPort(2); // use UART2

void motor_init(void);

void backward(void);
void forward(void);
void left(void);
void right(void);
void stop(void);

void setup() {
  // put your setup code here, to run once:
  motor_init();
  SerialPort.begin(9600);
  Serial.begin(9600);

}

void loop() {
  // put your main code here, to run repeatedly:
    // 检查是否有数据可读
  if (SerialPort.available() > 0) {
    // 读取一个字符
    char receivedChar = SerialPort.read();
    // 判断接收到的字符是否为 'F'
    if (receivedChar == 'F') {
      // 执行特定功能
      forward();
    }
    if (receivedChar == 'B') {
      // 执行特定功能
      backward();
    }
    if (receivedChar == 'L') {
      // 执行特定功能
      left();
    }
    if (receivedChar == 'R') {
      // 执行特定功能
      right();
    }      
    if (receivedChar == 'S') {
      // 执行特定功能
      stop();
    }
  }
}

void motor_init()
{
  ledcSetup(MOTOR3_IN1_CHANNEL,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(MOTOR3_IN1,MOTOR3_IN1_CHANNEL);
  ledcSetup(MOTOR3_IN2_CHANNEL,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(MOTOR3_IN2,MOTOR3_IN2_CHANNEL);  

  ledcSetup(MOTOR4_IN1_CHANNEL,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(MOTOR4_IN1,MOTOR4_IN1_CHANNEL);
  ledcSetup(MOTOR4_IN2_CHANNEL,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(MOTOR4_IN2,MOTOR4_IN2_CHANNEL);

  ledcSetup(MOTOR5_IN1_CHANNEL,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(MOTOR5_IN1,MOTOR5_IN1_CHANNEL);
  ledcSetup(MOTOR5_IN2_CHANNEL,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(MOTOR5_IN2,MOTOR5_IN2_CHANNEL);

  ledcSetup(MOTOR6_IN1_CHANNEL,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(MOTOR6_IN1,MOTOR6_IN1_CHANNEL);
  ledcSetup(MOTOR6_IN2_CHANNEL,PWM_FREQ,PWM_RESOLUTION);
  ledcAttachPin(MOTOR6_IN2,MOTOR6_IN2_CHANNEL); 
}

void forward()
{
  ledcWrite(MOTOR3_IN1_CHANNEL,255);    // 左前
  ledcWrite(MOTOR3_IN2_CHANNEL,0);

  ledcWrite(MOTOR4_IN1_CHANNEL,0);    // 右前
  ledcWrite(MOTOR4_IN2_CHANNEL,255);

  ledcWrite(MOTOR5_IN1_CHANNEL,255);    // 左后
  ledcWrite(MOTOR5_IN2_CHANNEL,0);

  ledcWrite(MOTOR6_IN1_CHANNEL,0);    // 右后
  ledcWrite(MOTOR6_IN2_CHANNEL,255);
}

void backward()
{
  ledcWrite(MOTOR3_IN1_CHANNEL,0);    // 左前
  ledcWrite(MOTOR3_IN2_CHANNEL,255);

  ledcWrite(MOTOR4_IN1_CHANNEL,255);    // 右前
  ledcWrite(MOTOR4_IN2_CHANNEL,0);

  ledcWrite(MOTOR5_IN1_CHANNEL,0);    // 左后
  ledcWrite(MOTOR5_IN2_CHANNEL,255);

  ledcWrite(MOTOR6_IN1_CHANNEL,255);    // 右后
  ledcWrite(MOTOR6_IN2_CHANNEL,0);
}

void left()
{
  ledcWrite(MOTOR3_IN1_CHANNEL,0);    // 左前
  ledcWrite(MOTOR3_IN2_CHANNEL,255);

  ledcWrite(MOTOR4_IN1_CHANNEL,0);    // 右前
  ledcWrite(MOTOR4_IN2_CHANNEL,255);

  ledcWrite(MOTOR5_IN1_CHANNEL,0);    // 左后
  ledcWrite(MOTOR5_IN2_CHANNEL,255);

  ledcWrite(MOTOR6_IN1_CHANNEL,0);    // 右后
  ledcWrite(MOTOR6_IN2_CHANNEL,255);  
}

void right()
{
  ledcWrite(MOTOR3_IN1_CHANNEL,255);    // 左前
  ledcWrite(MOTOR3_IN2_CHANNEL,0);

  ledcWrite(MOTOR4_IN1_CHANNEL,255);    // 右前
  ledcWrite(MOTOR4_IN2_CHANNEL,0);

  ledcWrite(MOTOR5_IN1_CHANNEL,255);    // 左后
  ledcWrite(MOTOR5_IN2_CHANNEL,0);

  ledcWrite(MOTOR6_IN1_CHANNEL,255);    // 右后
  ledcWrite(MOTOR6_IN2_CHANNEL,0);  
}

void stop()
{
  ledcWrite(MOTOR3_IN1_CHANNEL,0);    // 左前
  ledcWrite(MOTOR3_IN2_CHANNEL,0);

  ledcWrite(MOTOR4_IN1_CHANNEL,0);    // 右前
  ledcWrite(MOTOR4_IN2_CHANNEL,0);

  ledcWrite(MOTOR5_IN1_CHANNEL,0);    // 左后
  ledcWrite(MOTOR5_IN2_CHANNEL,0);

  ledcWrite(MOTOR6_IN1_CHANNEL,0);    // 右后
  ledcWrite(MOTOR6_IN2_CHANNEL,0);  
}
