/*
  前进、后退、左转、右转   四个button
  中间步进电机上升下降     1个button  stepper1
  盒子一出一入            1个        stepper2
  上升下降                1个        stepper3、stepper4
  舵机旋转180             1个     点一下旋转180 再点归位
  传送带                  1个     点一下开，点一下关闭          
*/

#define BLINKER_WIFI

#include <Blinker.h>
#include <stdint.h>
#include <HardwareSerial.h>  // 硬件串口

#define STEPPER1_STEP 25
#define STEPPER1_DIR 26
#define STEPPER2_STEP 33
#define STEPPER2_DIR 32
#define STEPPER3_STEP 27
#define STEPPER3_DIR 14
#define STEPPER4_STEP 2
#define STEPPER4_DIR 0
#define STEPPER5_STEP 4
#define STEPPER5_DIR 5

#define STEPPER1_STEP_CHANNEL 4
#define STEPPER2_STEP_CHANNEL 5
#define STEPPER3_STEP_CHANNEL 6
#define STEPPER4_STEP_CHANNEL 7
#define STEPPER5_STEP_CHANNEL 8

#define STEPPER_PWM_FREQ       1000
#define STEPPER_PWM_RESOLUTION 8

#define MOTOR1_IN1 22
#define MOTOR1_IN2 19
#define MOTOR2_IN1 23
#define MOTOR2_IN2 18

#define MOTOR1_IN1_CHANNEL 0
#define MOTOR1_IN2_CHANNEL 1
#define MOTOR2_IN1_CHANNEL 2
#define MOTOR2_IN2_CHANNEL 3

#define MOTOR_PWM_FREQ         1000
#define MOTOR_PWM_RESOLUTION   8

#define SERVO1 13
#define SERVO2 15
#define SERVO3 12

HardwareSerial SerialPort(2);  // use UART2

char auth[] = "0eb5bbed414b";
char ssid[] = "OnePlus";
char pswd[] = "99999999";

// 底盘运动
BlinkerButton ButtonForward("forward");
BlinkerButton ButtonBackward("backward");
BlinkerButton ButtonLeft("left");
BlinkerButton ButtonRight("right");

// 直流电机运动
BlinkerButton ButtonBelt("belt");  // 皮带

// 步进电机运动
BlinkerButton ButtonCenter("center");    // 中间开沟覆土装置升降
BlinkerButton ButtonBox1("box1");        // 运输箱子
BlinkerButton ButtonLifting("lifting");  // 升降装置

// 开沟覆土舵机
BlinkerButton ButtonCenterServo("servo");  // 开沟覆土舵机

void forward(const String& state);
void backward(const String& state);
void left(const String& state);
void right(const String& state);
void beltrun(const String& state);
void centerservo(const String& state);
void centerstepper(const String& state);
void box1(const String& state);
void lifting(const String& state);

void setup() {
  // put your setup code here, to run once:
  motor_init();
  SerialPort.begin(9600);

  // 初始化串口
  Serial.begin(115200);
  BLINKER_DEBUG.stream(Serial);
  BLINKER_DEBUG.debugAll();

  Blinker.begin(auth, ssid, pswd);
  // 底盘运动
  ButtonForward.attach(forward);
  ButtonBackward.attach(backward);
  ButtonLeft.attach(left);
  ButtonRight.attach(right);
  // 皮带运动
  ButtonBelt.attach(beltrun);
  // 舵机运动
  ButtonCenterServo.attach(centerservo);
  // 步进电机
  ButtonCenter.attach(centerstepper);
  ButtonBox1.attach(box1);
  ButtonLifting.attach(lifting);
}

void loop() {
  // put your main code here, to run repeatedly:
  Blinker.run();
}

void motor_init() {
  //步进电机
  ledcSetup(STEPPER1_STEP_CHANNEL, STEPPER_PWM_FREQ, STEPPER_PWM_RESOLUTION);
  ledcAttachPin(STEPPER1_STEP, STEPPER1_STEP_CHANNEL);
  pinMode(STEPPER1_DIR, OUTPUT);

  ledcSetup(STEPPER2_STEP_CHANNEL, STEPPER_PWM_FREQ, STEPPER_PWM_RESOLUTION);
  ledcAttachPin(STEPPER2_STEP, STEPPER2_STEP_CHANNEL);
  pinMode(STEPPER2_DIR, OUTPUT);

  ledcSetup(STEPPER3_STEP_CHANNEL, STEPPER_PWM_FREQ, STEPPER_PWM_RESOLUTION);
  ledcAttachPin(STEPPER3_STEP, STEPPER3_STEP_CHANNEL);
  pinMode(STEPPER3_DIR, OUTPUT);

  ledcSetup(STEPPER4_STEP_CHANNEL, STEPPER_PWM_FREQ, STEPPER_PWM_RESOLUTION);
  ledcAttachPin(STEPPER4_STEP, STEPPER4_STEP_CHANNEL);
  pinMode(STEPPER4_DIR, OUTPUT);

  ledcSetup(STEPPER5_STEP_CHANNEL, STEPPER_PWM_FREQ, STEPPER_PWM_RESOLUTION);
  ledcAttachPin(STEPPER5_STEP, STEPPER5_STEP_CHANNEL);
  pinMode(STEPPER5_DIR, OUTPUT);
  // 直流电机--皮带
  ledcSetup(MOTOR1_IN1_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR1_IN1, MOTOR1_IN1_CHANNEL);

  ledcSetup(MOTOR1_IN2_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR1_IN2, MOTOR1_IN2_CHANNEL);

  ledcSetup(MOTOR2_IN1_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR2_IN1, MOTOR2_IN1_CHANNEL);

  ledcSetup(MOTOR2_IN2_CHANNEL, MOTOR_PWM_FREQ, MOTOR_PWM_RESOLUTION);
  ledcAttachPin(MOTOR2_IN2, MOTOR2_IN2_CHANNEL);

  digitalWrite(STEPPER1_DIR,HIGH);
  digitalWrite(STEPPER2_DIR,HIGH);
  digitalWrite(STEPPER3_DIR,HIGH);
  digitalWrite(STEPPER4_DIR,HIGH);    
}

void forward(const String& state) {
  BLINKER_LOG("[forward] >> get button state:", state);
  if (state == BLINKER_CMD_BUTTON_PRESSED) {
    SerialPort.write('F');
  } else if (state == BLINKER_CMD_BUTTON_RELEASED) {
    SerialPort.write('S');
  }
}

void backward(const String& state) {
  BLINKER_LOG("[backward] >> get button state:", state);
  if (state == BLINKER_CMD_BUTTON_PRESSED) {
    SerialPort.write('B');
  } else if (state == BLINKER_CMD_BUTTON_RELEASED) {
    SerialPort.write('S');
  }
}

void left(const String& state) {
  BLINKER_LOG("[left] >> get button state:", state);
  if (state == BLINKER_CMD_BUTTON_PRESSED) {
    SerialPort.write('L');
  } else if (state == BLINKER_CMD_BUTTON_RELEASED) {
    SerialPort.write('S');
  }
}

void right(const String& state) {
  BLINKER_LOG("[right] >> get button state:", state);
  if (state == BLINKER_CMD_BUTTON_PRESSED) {
    SerialPort.write('R');
  } else if (state == BLINKER_CMD_BUTTON_RELEASED) {
    SerialPort.write('S');
  }
}

void beltrun(const String& state) {
  BLINKER_LOG("[beltrun] >> get button state:", state);
  static uint8_t beltrun1_cmd = 0;
  if (state == BLINKER_CMD_BUTTON_TAP) {
    beltrun1_cmd = !beltrun1_cmd;
    if (beltrun1_cmd == 0)  // 停止
    {
      ledcWrite(MOTOR1_IN1_CHANNEL,0);
      ledcWrite(MOTOR1_IN2_CHANNEL,255);
      ledcWrite(MOTOR2_IN1_CHANNEL,255);
      ledcWrite(MOTOR2_IN2_CHANNEL,0);
    } else if (beltrun1_cmd == 1)         // 运动
    {
      ledcWrite(MOTOR1_IN1_CHANNEL,255);
      ledcWrite(MOTOR1_IN2_CHANNEL,0);
      ledcWrite(MOTOR2_IN1_CHANNEL,0);
      ledcWrite(MOTOR2_IN2_CHANNEL,255);
    }
  }
  else if(state == BLINKER_CMD_BUTTON_PRESSED)
  {
    ledcWrite(MOTOR1_IN1_CHANNEL,0);
    ledcWrite(MOTOR1_IN2_CHANNEL,0);
    ledcWrite(MOTOR2_IN1_CHANNEL,0);
    ledcWrite(MOTOR2_IN2_CHANNEL,0);    
  }

}

void centerservo(const String& state) {
  BLINKER_LOG("[centerservo] >> get button state:", state);
  static uint8_t centerservo_cmd = 0;
  if (state == BLINKER_CMD_BUTTON_TAP) {
    centerservo_cmd = !centerservo_cmd;
    if (centerservo_cmd == 0)  // 转回0
    {
      
    } else if (centerservo_cmd == 1)  // 转180
    {

    }
  }
}

void centerstepper(const String& state) {
  BLINKER_LOG("[centerstepper] >> get button state:", state);
  static uint8_t centerstepper_cmd = 0;
  if (state == BLINKER_CMD_BUTTON_TAP) {
    centerstepper_cmd = !centerstepper_cmd;
    if (centerstepper_cmd == 0)  // 正转
    {
      digitalWrite(STEPPER1_DIR,HIGH);
    } else if (centerstepper_cmd == 1)  // 反转
    {
      digitalWrite(STEPPER1_DIR,LOW);
    }
  } else if (state == BLINKER_CMD_BUTTON_PRESSED)  // 转
  {
    ledcWrite(STEPPER1_STEP_CHANNEL,128);

  } else if (state == BLINKER_CMD_BUTTON_RELEASED)  // 停
  {
    ledcWrite(STEPPER1_STEP_CHANNEL,0);
  }
}

void box1(const String& state) {
  BLINKER_LOG("[box1] >> get button state:", state);
  static uint8_t box1_cmd = 0;
  if (state == BLINKER_CMD_BUTTON_TAP) {
    box1_cmd = !box1_cmd;
    if (box1_cmd == 0)  // 正转
    {
      digitalWrite(STEPPER2_DIR,LOW);
    } else if (box1_cmd == 1)  // 反转
    {
      digitalWrite(STEPPER2_DIR,HIGH);
    }
  } else if (state == BLINKER_CMD_BUTTON_PRESSED)  // 转
  {
    ledcWrite(STEPPER2_STEP_CHANNEL,128);
  } else if (state == BLINKER_CMD_BUTTON_RELEASED)  // 停
  {
    ledcWrite(STEPPER2_STEP_CHANNEL,0);
  }
}

void lifting(const String& state) {
  BLINKER_LOG("[lifting] >> get button state:", state);
  static uint8_t lifting_cmd = 0;
  if (state == BLINKER_CMD_BUTTON_TAP) {
    lifting_cmd = !lifting_cmd;
    if (lifting_cmd == 0)  // 正转
    {
      digitalWrite(STEPPER3_DIR,HIGH);
      digitalWrite(STEPPER4_DIR,HIGH);
    } else if (lifting_cmd == 1)  // 反转
    {
      digitalWrite(STEPPER3_DIR,LOW);
      digitalWrite(STEPPER4_DIR,LOW);
    }
  } else if (state == BLINKER_CMD_BUTTON_PRESSED)  // 转
  {
    ledcWrite(STEPPER3_STEP_CHANNEL,128);
    ledcWrite(STEPPER4_STEP_CHANNEL,128);
  } else if (state == BLINKER_CMD_BUTTON_RELEASED)  // 停
  {
    ledcWrite(STEPPER3_STEP_CHANNEL,0);
    ledcWrite(STEPPER4_STEP_CHANNEL,0);
  }
}
