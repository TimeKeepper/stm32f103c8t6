#define MenTion9_Servent

#ifdef MenTion2

#include "digital_io.h"           //包括数字IO控制头文件
#include "stm32f1xx_ll_gpio.h"    //包括GPIO控制头文件
#include <Arduino.h>              //包括Arduino核心头文件
#include "USBSerial.h"            //包括USB串口控制头文件 
#include "wiring_analog.h"
#include "wiring_digital.h"
#include <Adafruit_SSD1306.h> 

#define SerialSpeed 115200

#define LED1 PB12
#define LED2 PB13
#define LED3 PB14
#define LED4 PB15
#define LED5 PA8
#define LED6 PB3
#define LED7 PB4
#define LED8 PB5

#define KEY1 PA3
#define KEY2 PC14

#define BEEP PA1

/*
实现以下功能：
1.按键控制LED，每按下一次按键触发中断，在中断函数中修改要亮起的LED
2.按键控制蜂鸣器，每按下一次按键触发一次中断，在中断函数中开关蜂鸣器
*/

void setup() {
  SerialUSB.begin(SerialSpeed);               //设置USB模拟串口波特率
  SerialUSB.printf("Setup Complete, SerialSpeed %d\n", SerialSpeed);  //串口初始化成功打印信息
  pinMode(LED1, OUTPUT);                      //设置LED1为输出模式
  pinMode(LED2, OUTPUT);                      //设置LED2为输出模式
  pinMode(LED3, OUTPUT);                      //设置LED3为输出模式
  pinMode(LED4, OUTPUT);                      //设置LED4为输出模式
  pinMode(LED5, OUTPUT);                      //设置LED5为输出模式
  pinMode(LED6, OUTPUT);                      //设置LED6为输出模式
  pinMode(LED7, OUTPUT);                      //设置LED7为输出模式
  pinMode(LED8, OUTPUT);                      //设置LED8为输出模式
  digitalWrite(LED1, LOW);                  
  digitalWrite(LED2, LOW);                  
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  digitalWrite(LED5, LOW);
  digitalWrite(LED6, LOW);
  digitalWrite(LED7, LOW);
  digitalWrite(LED8, LOW);

  pinMode(KEY1, INPUT_PULLDOWN);                //设置KEY1为输入模式，上拉输入
  pinMode(KEY2, INPUT_PULLDOWN);                //设置KEY2为输入模式，上拉输入
  pinMode(BEEP, OUTPUT);                      //设置BEEP为输出模式
  attachInterrupt(KEY1, [](){                 //设置KEY1中断函数
    static uint32_t LED_NUM = 0;              //定义静态变量LED_NUM
    LED_NUM++;                                //LED_NUM自增
    if(LED_NUM > 8) LED_NUM = 1;              //LED_NUM大于8时，LED_NUM置1
    uint32_t LED_PIN = 0;                     //定义LED_PIN变量
    switch(LED_NUM){
      case 1: LED_PIN = LED1; break;          //根据LED_NUM值，设置LED_PIN
      case 2: LED_PIN = LED2; break;
      case 3: LED_PIN = LED3; break;
      case 4: LED_PIN = LED4; break;
      case 5: LED_PIN = LED5; break;
      case 6: LED_PIN = LED6; break;
      case 7: LED_PIN = LED7; break;
      case 8: LED_PIN = LED8; break;
    } 
    digitalWrite(LED_PIN, !digitalRead(LED_PIN));  //取反LED_PIN引脚电平
  }, RISING);                                //设置中断触发方式为下降沿触发
  attachInterrupt(KEY2, [](){                 //设置KEY2中断函数
    static uint32_t BEEP_STATE = 0;           //定义静态变量BEEP_STATE
    BEEP_STATE = !BEEP_STATE;                 //BEEP_STATE取反
    analogWrite(BEEP, BEEP_STATE * 128);      //设置BEEP引脚占空比
  }, RISING);                                //设置中断触发方式为下降沿触发
}

void loop() {
  SerialUSB.println("Hit loop");  //打印信息
  delay(1000);                    //延时1s
}


#endif

#ifdef MenTion3

#include "digital_io.h"           //包括数字IO控制头文件
#include "stm32f1xx_ll_gpio.h"    //包括GPIO控制头文件
#include <Arduino.h>              //包括Arduino核心头文件
#include <cstdint>
#include "USBSerial.h"            //包括USB串口控制头文件 
#include "wiring_analog.h"
#include "wiring_constants.h"
#include "wiring_digital.h"
#include "wiring_time.h"
#include <Adafruit_SSD1306.h>

#define SerialSpeed 115200

#define LED1 PB12
#define LED2 PB13
#define LED3 PB14
#define LED4 PB15
#define LED5 PA8
#define LED6 PB3
#define LED7 PB4
#define LED8 PB5

#define KEY1 PA3
#define KEY2 PC14

#define BEEP PA1

/*
实现以下功能：
1.按键控制LED，每按下一次按键触发中断，在中断函数中修改要亮起的LED
2.按键控制蜂鸣器，每按下一次按键触发一次中断，在中断函数中开关蜂鸣器
*/

static uint32_t BEEP_STATE = 0;           //定义静态变量BEEP_STATE
    static uint32_t LED_NUM = 0;              //定义静态变量LED_NUM

void setup() {
  SerialUSB.begin(SerialSpeed);               //设置USB模拟串口波特率
  SerialUSB.printf("Setup Complete, SerialSpeed %d\n", SerialSpeed);  //串口初始化成功打印信息
  pinMode(LED1, OUTPUT);                      //设置LED1为输出模式
  pinMode(LED2, OUTPUT);                      //设置LED2为输出模式
  pinMode(LED3, OUTPUT);                      //设置LED3为输出模式
  pinMode(LED4, OUTPUT);                      //设置LED4为输出模式
  pinMode(LED5, OUTPUT);                      //设置LED5为输出模式
  pinMode(LED6, OUTPUT);                      //设置LED6为输出模式
  pinMode(LED7, OUTPUT);                      //设置LED7为输出模式
  pinMode(LED8, OUTPUT);                      //设置LED8为输出模式
  digitalWrite(LED1, LOW);                  
  digitalWrite(LED2, LOW);                  
  digitalWrite(LED3, LOW);
  digitalWrite(LED4, LOW);
  digitalWrite(LED5, LOW);
  digitalWrite(LED6, LOW);
  digitalWrite(LED7, LOW);
  digitalWrite(LED8, LOW);
  
  pinMode(KEY1, INPUT_PULLDOWN);                //设置KEY1为输入模式，上拉输入
  pinMode(KEY2, INPUT_PULLDOWN);                //设置KEY2为输入模式，上拉输入
  pinMode(BEEP, OUTPUT);                      //设置BEEP为输出模式
  attachInterrupt(KEY1, [](){                 //设置KEY1中断函数
    static uint32_t local_millis = millis();
    //通过判断millis值来进行按键消抖
    bool judge = millis() - local_millis < 200;
    local_millis = millis();
    if(judge) return;
    LED_NUM++;                                //LED_NUM自增
    if(LED_NUM > 8) LED_NUM = 1;              //LED_NUM大于8时，LED_NUM置1
  }, RISING);                                //设置中断触发方式为下降沿触发
  attachInterrupt(KEY2, [](){                 //设置KEY2中断函数
    static uint32_t local_millis = millis();
    //通过判断millis值来进行按键消抖
    bool judge = millis() - local_millis < 200;
    local_millis = millis();
    if(judge) return;
    BEEP_STATE++;
    if(BEEP_STATE > 7) BEEP_STATE = 0;        //根据BEEP_STATE值，设置不同的蜂鸣器音调
    //设置pwm输出分辨率为10位(1024)
    analogWriteResolution(10);
    analogWrite(BEEP, BEEP_STATE * 128);      //设置BEEP引脚PWM输出
  }, RISING);                                //设置中断触发方式为下降沿触发
}

void key2_delay(void){
  //根据BEEP_STATE值，设置不同的延时时间，越高则延时越短，实现越快的跑马灯
  switch(BEEP_STATE){
    case 0: delay(100); break;
    case 1: delay(80); break;
    case 2: delay(60); break;
    case 3: delay(40); break;
    case 4: delay(20); break;
    case 5: delay(10); break;
    case 6: delay(5); break;
    case 7: delay(1); break;
  }
}

void LED_RUNtype(uint32_t type){
  //根据type值，设置不同的LED跑马灯模式
  switch(type){
    case 1:{
      //LED从左亮到右再从右亮到左
      for(uint32_t i = 0; i < 8; i++){
        switch(i){
          case 0: digitalWrite(LED1, HIGH); break;
          case 1: digitalWrite(LED2, HIGH); break;
          case 2: digitalWrite(LED3, HIGH); break;
          case 3: digitalWrite(LED4, HIGH); break;
          case 4: digitalWrite(LED5, HIGH); break;
          case 5: digitalWrite(LED6, HIGH); break;
          case 6: digitalWrite(LED7, HIGH); break;
          case 7: digitalWrite(LED8, HIGH); break;
        }
        key2_delay();
      }
      for(uint32_t i = 0; i < 8; i++){
        switch(i){
          case 0: digitalWrite(LED8, LOW); break;
          case 1: digitalWrite(LED7, LOW); break;
          case 2: digitalWrite(LED6, LOW); break;
          case 3: digitalWrite(LED5, LOW); break;
          case 4: digitalWrite(LED4, LOW); break;
          case 5: digitalWrite(LED3, LOW); break;
          case 6: digitalWrite(LED2, LOW); break;
          case 7: digitalWrite(LED1, LOW); break;
        }
        key2_delay();
      }
      break;
      }
    case 2:{
      //LED从左亮到右再从右亮到左，中间LED一直亮
      for(uint32_t i = 0; i < 8; i++){
        switch(i){
          case 0: digitalWrite(LED1, HIGH); break;
          case 1: digitalWrite(LED2, HIGH); break;
          case 2: digitalWrite(LED3, HIGH); break;
          case 3: digitalWrite(LED4, HIGH); break;
          case 4: digitalWrite(LED5, HIGH); break;
          case 5: digitalWrite(LED6, HIGH); break;
          case 6: digitalWrite(LED7, HIGH); break;
          case 7: digitalWrite(LED8, HIGH); break;
        }
        key2_delay();
      }
      for(uint32_t i = 0; i < 8; i++){
        switch(i){
          case 0: digitalWrite(LED8, LOW); break;
          case 1: digitalWrite(LED7, LOW); break;
          case 2: digitalWrite(LED6, LOW); break;
          case 3: digitalWrite(LED5, LOW); break;
          case 4: digitalWrite(LED4, LOW); break;
          case 5: digitalWrite(LED3, LOW); break;
          case 6: digitalWrite(LED2, LOW); break;
          case 7: digitalWrite(LED1, LOW); break;
        }
        key2_delay();
      }
      digitalWrite(LED5, HIGH);
      break;
      }
    case 3:{
      //LED沙漏
      for(uint32_t i = 0; i < 4; i++){
        switch(i){
          case 0: digitalWrite(LED1, HIGH); digitalWrite(LED8, HIGH); key2_delay();break;
          case 1: digitalWrite(LED2, HIGH); digitalWrite(LED7, HIGH); key2_delay();break;
          case 2: digitalWrite(LED3, HIGH); digitalWrite(LED6, HIGH); key2_delay();break;
          case 3: digitalWrite(LED4, HIGH); digitalWrite(LED5, HIGH); key2_delay();break;
        }
        key2_delay();
      }
      for(uint32_t i = 0; i < 4; i++){
        switch(i){
          case 0: digitalWrite(LED1, LOW); digitalWrite(LED8, LOW); key2_delay();break;
          case 1: digitalWrite(LED2, LOW); digitalWrite(LED7, LOW); key2_delay();break;
          case 2: digitalWrite(LED3, LOW); digitalWrite(LED6, LOW); key2_delay();break;
          case 3: digitalWrite(LED4, LOW); digitalWrite(LED5, LOW); key2_delay();break;
        }
        key2_delay();
      }
      break;
    }
    case 4:{
      //LED沙漏，“沙粒”从LED1落到LED2，此时LED2亮起，LED1熄灭，以此类推逐渐落到LED8，然后占据LED8的位置，然后再次有一个沙粒从LED1落到LED7，以此类推，指导LED被填满则清空
      for(uint32_t i = 8; i; i--){
        for(uint32_t j = 0; j < i; j++){
          switch(i){
            case 0: digitalWrite(LED1, HIGH); key2_delay();break;
            case 1: digitalWrite(LED2, HIGH); digitalWrite(LED1, HIGH); key2_delay();break;
            case 2: digitalWrite(LED3, HIGH); digitalWrite(LED2, HIGH); key2_delay();break;
            case 3: digitalWrite(LED4, HIGH); digitalWrite(LED3, HIGH); key2_delay();break;
            case 4: digitalWrite(LED5, HIGH); digitalWrite(LED4, HIGH); key2_delay();break;
            case 5: digitalWrite(LED6, HIGH); digitalWrite(LED5, HIGH); key2_delay();break;
            case 6: digitalWrite(LED7, HIGH); digitalWrite(LED6, HIGH); key2_delay();break;
            case 7: digitalWrite(LED8, HIGH); digitalWrite(LED7, HIGH); key2_delay();break;
          }
          key2_delay();
        }
      }
    case 5:{
      for(int32_t i = 7; i+1; i--){
        switch(i){
          case 0: digitalWrite(LED1, HIGH); break;
          case 1: digitalWrite(LED2, HIGH); break;
          case 2: digitalWrite(LED3, HIGH); break;
          case 3: digitalWrite(LED4, HIGH); break;
          case 4: digitalWrite(LED5, HIGH); break;
          case 5: digitalWrite(LED6, HIGH); break;
          case 6: digitalWrite(LED7, HIGH); break;
          case 7: digitalWrite(LED8, HIGH); break;
        }
        key2_delay();
      }
      for(int32_t i = 7; i+1; i--){
        switch(i){
          case 0: digitalWrite(LED8, LOW); break;
          case 1: digitalWrite(LED7, LOW); break;
          case 2: digitalWrite(LED6, LOW); break;
          case 3: digitalWrite(LED5, LOW); break;
          case 4: digitalWrite(LED4, LOW); break;
          case 5: digitalWrite(LED3, LOW); break;
          case 6: digitalWrite(LED2, LOW); break;
          case 7: digitalWrite(LED1, LOW); break;
        }
        key2_delay();
      }
      break;
    }
    case 6:{
      for(int32_t i = 7; i+1; i--){
        switch(i){
          case 0: digitalWrite(LED1, HIGH); break;
          case 1: digitalWrite(LED2, HIGH); break;
          case 2: digitalWrite(LED3, HIGH); break;
          case 3: digitalWrite(LED4, HIGH); break;
          case 4: digitalWrite(LED5, HIGH); break;
          case 5: digitalWrite(LED6, HIGH); break;
          case 6: digitalWrite(LED7, HIGH); break;
          case 7: digitalWrite(LED8, HIGH); break;
        }
        key2_delay();
      }
      for(int32_t i = 7; i+1; i--){
        switch(i){
          case 0: digitalWrite(LED8, LOW); break;
          case 1: digitalWrite(LED7, LOW); break;
          case 2: digitalWrite(LED6, LOW); break;
          case 3: digitalWrite(LED5, LOW); break;
          case 4: digitalWrite(LED4, LOW); break;
          case 5: digitalWrite(LED3, LOW); break;
          case 6: digitalWrite(LED2, LOW); break;
          case 7: digitalWrite(LED1, LOW); break;
        }
        key2_delay();
      }
      digitalWrite(LED5, HIGH);
      break;
    }
  }
}
}

void loop() {
  LED_RUNtype(LED_NUM);
}

#endif

#ifdef MenTion4

#include "HardwareTimer.h"
#include "wiring_analog.h"
#include "wiring_constants.h"
#include "wiring_digital.h"
#include "wiring_time.h"
#include <Arduino.h>              //包括Arduino核心头文件
#include <Adafruit_VL53L0X.h>     //包括VL53L0X头文件
#include <Adafruit_SSD1306.h>
#include <USBSerial.h>
#include <Wire.h>
#include <cstdint>

#define SerialSpeed 115200

#define LED_RED PB3
#define LED_GREEN PB5
#define LED_YELLO PB4

#define KEY PA3

#define BEEP PA1

#define VX_SCL PB6
#define VX_SDA PB7

/*
实现以下功能：
1.单片机控制红绿灯循环亮灭，红灯30秒后黄灯亮，黄灯3秒后绿灯亮，绿灯30秒后黄灯亮，黄灯3秒后红灯亮，如此循环往复
2.在红灯亮起期间，“行人”不允许通过，激光测距模块会检测与行人的距离，如果进入警戒范围内，则发出警报使蜂鸣器响起。
3.在黄灯亮起期间，“行人”和“车辆”应当加速通过，此时若检测到有行人或车辆通过。蜂鸣器将以较低的音调响起
4.为方便工作人员对后续红绿灯的维护，按下按键会触发外部中断并改变红绿灯的模式，
模式1也就是默认模式，红绿灯正常工作，模式2红绿灯熄灭，模式3红绿灯常亮，模式4红绿灯熄灭并且蜂鸣器以高音调响起
此外还有每次复位后会进入的模式0，此模式下单片机不做任何工作，需要按下按钮来启动整个程序
*/

static uint32_t LED_MODE = 0;
  static uint32_t current_LED_MODE = 0;
static uint32_t Timer_CNT = 1;

TIM_TypeDef *Instance = TIM1;
HardwareTimer *Timer = new HardwareTimer(Instance);

enum LED_color{
  RED = 0,
  GREEN = 1,
  YELLO = 2,
  DARK = 3,
  LIGHT = 4
};

void LED_Light(LED_color LED);

void SwitchMode(uint32_t LED_MODE);

void EXTI_CBHandle(void);

void Timer_CBHandle(void);

void Lighting_loop(void);

Adafruit_VL53L0X lox = Adafruit_VL53L0X();  //定义一个VL53L0X对象

void setup(){
    pinMode(LED_RED,OUTPUT);
    pinMode(LED_GREEN,OUTPUT);
    pinMode(LED_YELLO,OUTPUT);
    LED_Light(DARK);

    pinMode(KEY,INPUT_PULLDOWN);
    attachInterrupt(KEY,EXTI_CBHandle,RISING);
    pinMode(BEEP,OUTPUT);
    analogWriteResolution(10);

    SerialUSB.begin(SerialSpeed);

    //因为stm32f103只有一个硬件IIC，所以不能自己定义一个新的ToWire对象，只能使用Wire对象，不然就会HardWare_Fault
    Wire.setSDA(VX_SDA);
    Wire.setSCL(VX_SCL);
    Wire.begin();
    while(!lox.begin(VL53L0X_I2C_ADDR,false,&Wire));  //初始化VL53L0X对象，将IIC对象地址传入

    // lox.startRangeContinuous();
    
    //打开定时器
    Timer->setOverflow(1,HERTZ_FORMAT);
    Timer->attachInterrupt(Timer_CBHandle);

    SerialUSB.begin(SerialSpeed);
    SerialUSB.println("Setup_Down");      //USB虚拟串口打印信息
}

void vx_loop(void){
  //激光测距模块检测到有行人或车辆通过。蜂鸣器将以较低的音调响起
  //激光测距模块检测到与行人的距离，如果进入警戒范围内，则发出警报使蜂鸣器响起。
  static uint32_t local_millis = millis();
  if(LED_MODE != 1) return;
  if(millis() - local_millis < 10) return;
  local_millis = millis();

  VL53L0X_RangingMeasurementData_t measure;
  lox.rangingTest(&measure,false);
  if(lox.readRange() < 100 && current_LED_MODE != 2){
    //检测到有行人或车辆通过
    analogWrite(BEEP,current_LED_MODE == 0 ? 512 : 100);
  }else{
    analogWrite(BEEP,0);
  }

}

void loop(){
  Lighting_loop();
  vx_loop();
}

void LED_Light(LED_color LED){
  switch(LED){
    case RED:{
      digitalWrite(LED_RED,HIGH);
      digitalWrite(LED_GREEN,LOW);
      digitalWrite(LED_YELLO,LOW);
      break;
    }
    case GREEN:{
      digitalWrite(LED_RED,LOW);
      digitalWrite(LED_GREEN,HIGH);
      digitalWrite(LED_YELLO,LOW);
      break;
    }
    case YELLO:{
      digitalWrite(LED_RED,LOW);
      digitalWrite(LED_GREEN,LOW);
      digitalWrite(LED_YELLO,HIGH);
      break;
    }
    case DARK:{
      digitalWrite(LED_RED,LOW);
      digitalWrite(LED_GREEN,LOW);
      digitalWrite(LED_YELLO,LOW);
      break;
    }
    case LIGHT:{
      digitalWrite(LED_RED,HIGH);
      digitalWrite(LED_GREEN,HIGH);
      digitalWrite(LED_YELLO,HIGH);
      break;
    }
    default: break;
  }
}

void Out_Normal_Clear(void){
  analogWrite(BEEP,0);
  Timer->pause();
  LED_Light(DARK);
}

void In_Normal_Begin(void){
  Timer->resume();
  LED_Light(RED);
  Timer_CNT = 0;
}

void SwitchMode(uint32_t LED_MODE){
  //根据LED_MODE的值来控制LED的亮灭
  SerialUSB.println("SwitchMode:" + String(LED_MODE));
  switch(LED_MODE){
    case 1:{
      //模式1,通过定时中断实现红绿灯的循环亮灭
      In_Normal_Begin();
      break;
    }
    case 2:
      Out_Normal_Clear();
      LED_Light(DARK);
      break;
    case 3:
      Out_Normal_Clear();
      LED_Light(LIGHT);
      break;
    case 4:
      Out_Normal_Clear();
      LED_Light(DARK);
      analogWrite(BEEP,512);
      break;
    default: break;
  }
}

void EXTI_CBHandle(void){
  //外部中断回调函数
  //当按键按下,修改LED_MODE
  //首先进行消抖
  static uint32_t local_millis = millis();
  if(millis() - local_millis < 100) return;
  local_millis = millis();
  
  if(++LED_MODE > 4) LED_MODE = 1;
  SwitchMode(LED_MODE);
}

void Timer_CBHandle(void){
  //定时器中断回调函数
  //每次进入该函数,使Timer_CNT加一
  if(++Timer_CNT > 1000) Timer_CNT = 1;
}

void Lighting_loop(void){
  switch(current_LED_MODE){
    case 0:{
      if(Timer_CNT < 30) return;
      current_LED_MODE = 1;
      LED_Light(YELLO);
      Timer_CNT = 0;
    }
    case 1:{
      if(Timer_CNT < 3) return;
      current_LED_MODE = 2;
      LED_Light(GREEN);
      Timer_CNT = 0;
    }
    case 2:{
      if(Timer_CNT < 15) return;
      current_LED_MODE = 3;
      LED_Light(YELLO);
      Timer_CNT = 0;
    }
    case 3:{
      if(Timer_CNT < 3) return;
      current_LED_MODE = 0;
      LED_Light(RED);
      Timer_CNT = 0;
    }
    default: break;
  }
}

#endif

#ifdef MenTion5
#include <Arduino.h>
#include <Keypad.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
{'g','f','e', 'd'},
{'c','b','a','9'},
{'8','7','6','5'},
{'4','3','2','1'}
};
byte rowPins[ROWS] = {PB0, PB1, PB10, PB11}; //connect to the row pinouts of the kpd
byte colPins[COLS] = {PA7, PA6, PA5, PA4}; //connect to the column pinouts of the kpd

Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

#define SCL PB6
#define SDA PB7
Adafruit_SSD1306 OLED = Adafruit_SSD1306(128,64,&Wire);

#define BUZZER PA1

#define do 262
#define re 294
#define mi 330
#define fa 349
#define sol 392
#define la 440
#define si 494
#define do1 523
#define re1 587
#define stop 1

void buzzer_init(){
  pinMode(BUZZER,OUTPUT);
  analogWrite(BUZZER,0);
}

void play(int note){
  if(note == stop){
    analogWrite(BUZZER,0);
    return;
  }
  analogWriteFrequency(note);
  analogWrite(BUZZER,128);
}

void setup(){
  buzzer_init();

  Wire.setSCL(SCL);
  Wire.setSDA(SDA);
  Wire.begin();
  OLED.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  OLED.setTextSize(1);
  OLED.setTextColor(WHITE);
  OLED.setCursor(0,0);
  OLED.display();
  OLED.clearDisplay();
  delay(1000);
}

void loop(){
  if (kpd.getKeys()){
    OLED.clearDisplay();
    OLED.setCursor(0, 0);
    for (int i=0; i<LIST_MAX; i++){
      if (kpd.key[i].stateChanged){
        OLED.println(kpd.key[i].kchar);
        switch (kpd.key[i].kstate){
          case PRESSED:
            OLED.print("Pressed: ");
            switch (kpd.key[i].kchar){
              case '1':play(do);break;
              case '2':play(re);break;
              case '3':play(mi);break;
              case '4':play(fa);break;
              case '5':play(sol);break;
              case '6':play(la);break;
              case '7':play(si);break;
              case '8':play(do1);break;
              case '9':play(re1);break;
              case 'a':play(stop);break;
              case 'b':play(stop);break;
              case 'c':play(stop);break;
              case 'd':play(stop);break;
              case 'e':play(stop);break;
              case 'f':play(stop);break;
              case 'g':play(stop);break;
            }
            break;
          case HOLD:
            OLED.print("Hold: ");
            break;
          case RELEASED:
            OLED.print("Released: ");
            play(stop);
            break;
          case IDLE:
            OLED.print("Idle: ");
            break;
        }
      }
    }
  }
  OLED.display();
}
#endif

#ifdef MenTion6
#include <Arduino.h>
#include <Keypad.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <TM1637.h>

const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
{'=','c','/', '*'},
{'-','+','0','9'},
{'8','7','6','5'},
{'4','3','2','1'}
};
byte rowPins[ROWS] = {PB0, PB1, PB10, PB11}; //connect to the row pinouts of the kpd
byte colPins[COLS] = {PA7, PA6, PA5, PA4}; //connect to the column pinouts of the kpd

Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

TM1637 tm(PB8, PB9);

//使用矩阵键盘和数码管实现一个计算器

//创建一个整数存储当前的数字
int32_t num = 0;

void add_char(char ch){
  //将当前数字除以10，再加上ch对应的数字乘以1000
  num = (num * 10) + ((ch - '0')*1);
  //显示输入的数字，并补0到四位
  tm.display(num);
}

void clear(void){
  //将当前数字清零
  num = 0;
}

bool equal_jump(char ch){
  if(ch == '=') return true;
  return false;
}

void num_cut(void){
  num%=10000;
}

//使用一个函数实现加法
void add(void){
  tm.clearScreen();
  int32_t local_num = 0;
  for(int i = 0; i < 4; i++){
    char ch = kpd.waitForKey();
    if(ch=='=') break;
    local_num = local_num * 10 + (ch - '0');
    tm.display(local_num);
  }
  num += local_num;
  num_cut();
  tm.clearScreen();
  tm.display(num);
  clear();
  while(!(kpd.waitForKey()=='='));
  tm.clearScreen();
}

//使用一个函数实现减法
void sub(void){
  tm.clearScreen();
  int32_t local_num = 0;
  for(int i = 0; i < 4; i++){
    char ch = kpd.waitForKey();
    if(ch=='=') break;
    local_num = local_num * 10 + (ch - '0');
    tm.display(local_num);
  }
  num -= local_num;
  num_cut();
  tm.clearScreen();
  tm.display(num);
  clear();
  while(!(kpd.waitForKey()=='='));
  tm.clearScreen();
}

//使用一个函数实现乘法
void mul(void){
  tm.clearScreen();
  int32_t local_num = 0;
  for(int i = 0; i < 4; i++){
    char ch = kpd.waitForKey();
    if(ch=='=') break;
    local_num = local_num * 10 + (ch - '0');
    tm.display(local_num);
  }
  num *= local_num;
  num_cut();
  tm.clearScreen();
  tm.display(num);
  clear();
  while(!(kpd.waitForKey()=='='));
  tm.clearScreen();
}

//使用一个函数实现除法
void div(void){
  tm.clearScreen();
  int32_t local_num = 0;
  for(int i = 0; i < 4; i++){
    char ch = kpd.waitForKey();
    if(ch=='=') break;
    local_num = local_num * 10 + (ch - '0');
    tm.display(local_num);
  }
  float local_float_num = (float)num/local_num;
  num /= local_num;
  num_cut();
  tm.clearScreen();
  tm.display(local_float_num);
  clear();
  while(!(kpd.waitForKey()=='='));
  tm.clearScreen();
}

void setup(){
  tm.begin();
  tm.setBrightness(3);
}

void loop(){
  char ch = kpd.waitForKey();
  switch(ch){
    case '0':
    case '1':
    case '2':
    case '3':
    case '4':
    case '5':
    case '6':
    case '7':
    case '8':
    case '9': add_char(ch); break;
    case '+': add(); break;
    case '-': sub(); break;
    case '*': mul(); break;
    case '/': div(); break;
    case 'c': clear(); tm.clearScreen(); break;
    case '=': break;
    default: break;
  }
  num_cut();
}
#endif

#ifdef MenTion7
#include "wiring_time.h"
#include <Arduino.h>
#include <Keypad.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>
#include <USBSerial.h>

/*
实验七、PC串口监控系统设计
要求：
（1）创建KeilC工程，使开发板上电即显示0x55在PC串口工具对话框中；
（2）编写代码，使开发板上电即显示学生学号在PC串口工具对话框中；
（3）编写代码，使开发板的按键键值即时显示在PC串口工具对话框中； 
（4）编写代码，使PC串口工具对话框中输入5条指令均能获得开发板不同的回应内容。
（4*）编写代码，使VL53L0x芯片的数据有序显示在PC串口工具对话框中。
*/

#define VX_SCL PB6
#define VX_SDA PB7
Adafruit_VL53L0X lox = Adafruit_VL53L0X();  //定义一个VL53L0X对象

const byte ROWS = 4; //four rows
const byte COLS = 4; //three columns
char keys[ROWS][COLS] = {
{'=','c','/', '*'},
{'-','+','0','9'},
{'8','7','6','5'},
{'4','3','2','1'}
};
byte rowPins[ROWS] = {PB0, PB1, PB10, PB11}; //connect to the row pinouts of the kpd
byte colPins[COLS] = {PA7, PA6, PA5, PA4}; //connect to the column pinouts of the kpd

Keypad kpd = Keypad( makeKeymap(keys), rowPins, colPins, ROWS, COLS );

void setup(){
  SerialUSB.begin(115200);
  delay(10000);
  Wire.setSDA(VX_SDA);
  Wire.setSCL(VX_SCL);
  Wire.begin();
  while(!lox.begin(VL53L0X_I2C_ADDR,false,&Wire));  //初始化VL53L0X对象，将IIC对象地址传入

  SerialUSB.println("0x55");
  SerialUSB.println("2022280542");
  SerialUSB.setTimeout(3);
}

void loop(){
  static uint32_t loc_millis = millis();
  static bool is_VL_open = false;

  String str = SerialUSB.readStringUntil('\n');
  if(str == "test1") SerialUSB.println("Ok,you have input test1");
  else if(str == "test2") SerialUSB.println("you have input test2 this time.What's next?");
  else if(str == "test3") SerialUSB.println("so we got test3 huh?");
  else if(str == "test4") SerialUSB.println("exactly test4,then?");
  else if(str == "test5") SerialUSB.println("test mention accomplished");
  else if(str == "VL_Open") is_VL_open = true;
  else if(str == "VL_Close") is_VL_open = false;

  if(loc_millis - millis() > 300 && is_VL_open){
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure,false);
    if(measure.RangeStatus != 4) SerialUSB.println(measure.RangeMilliMeter);
    else SerialUSB.println("out of range");
  }

  char key = kpd.getKey();
  if(key == NO_KEY) return;
  SerialUSB.println(key);
}
#endif

#ifdef MenTion8
#include "wiring_time.h"
#include <Arduino.h>
#include <Keypad.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>
#include <USBSerial.h>
#include <cstdint>
#include "stm32yyxx_ll_adc.h"

#define VX_SCL PB6
#define VX_SDA PB7
Adafruit_VL53L0X lox = Adafruit_VL53L0X();  //定义一个VL53L0X对象

#define CALX_TEMP 25

#define VTEMP     1430
#define AVG_SLOPE 4300
#define VREFINT   1200

/* Analog read resolution */
#define LL_ADC_RESOLUTION LL_ADC_RESOLUTION_12B
#define ADC_RANGE 4096

static int32_t readVref()
{
  return (VREFINT * ADC_RANGE / analogRead(AVREF)); // ADC sample to mV
}

static int32_t readTempSensor(int32_t VRef)
{
  return (__LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(AVG_SLOPE, VTEMP, CALX_TEMP, VRef, analogRead(ATEMP), LL_ADC_RESOLUTION));
}

static int32_t readVoltage(int32_t VRef, uint32_t pin)
{
  return (__LL_ADC_CALC_DATA_TO_VOLTAGE(VRef, analogRead(pin), LL_ADC_RESOLUTION));
}

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  SerialUSB.begin(115200);
  // SerialUSB.setTimeout(UINT32_MAX);
  analogReadResolution(12);

  Wire.setSDA(VX_SDA);
  Wire.setSCL(VX_SCL);
  Wire.begin();
  while(!lox.begin(VL53L0X_I2C_ADDR,false,&Wire));  //初始化VL53L0X对象，将IIC对象地址传入
}


void loop() {
  String command = SerialUSB.readStringUntil('\n');

  if(command == "ls") {
    SerialUSB.println("VL_msg    VRef    Temp    V_A0");
  }
  else if(command == "VL_msg"){
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure,false);
    if(measure.RangeStatus != 4) SerialUSB.println(measure.RangeMilliMeter);
    else SerialUSB.println("out of range");
  }
  else if(command == "VRef") {
    int32_t VRef = readVref();
    SerialUSB.printf("VRef(mv)= %i", VRef);
  }
  else if(command == "V_A0"){
    int32_t VRef = readVref();
    SerialUSB.printf("V_A0= %i", readVoltage(VRef, A0));
  }
  else if(command == "Temp"){
    int32_t VRef = readVref();
    SerialUSB.printf("Temp(°C)= %i", readTempSensor(VRef));
  }
  else if(command == "") return;
  else 
    SerialUSB.println("unknown command");
}
#endif

#ifdef MenTion9_Servent

#include "HardwareSerial.h"
#include "wiring_time.h"
#include <Arduino.h>
#include <Keypad.h>
#include <Wire.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_VL53L0X.h>
#include <cstdint>
#include "stm32yyxx_ll_adc.h"

#define VX_SCL PB6
#define VX_SDA PB7
Adafruit_VL53L0X lox = Adafruit_VL53L0X();  //定义一个VL53L0X对象

#define CALX_TEMP 25

#define VTEMP     1430
#define AVG_SLOPE 4300
#define VREFINT   1200

/* Analog read resolution */
#define LL_ADC_RESOLUTION LL_ADC_RESOLUTION_12B
#define ADC_RANGE 4096

static int32_t readVref()
{
  return (VREFINT * ADC_RANGE / analogRead(AVREF)); // ADC sample to mV
}

static int32_t readTempSensor(int32_t VRef)
{
  return (__LL_ADC_CALC_TEMPERATURE_TYP_PARAMS(AVG_SLOPE, VTEMP, CALX_TEMP, VRef, analogRead(ATEMP), LL_ADC_RESOLUTION));
}

static int32_t readVoltage(int32_t VRef, uint32_t pin)
{
  return (__LL_ADC_CALC_DATA_TO_VOLTAGE(VRef, analogRead(pin), LL_ADC_RESOLUTION));
}

HardwareSerial Serial1(PA10, PA9);

// the setup routine runs once when you press reset:
void setup() {
  // initialize serial communication at 9600 bits per second:
  Serial1.begin(115200);
  analogReadResolution(12);

  Wire.setSDA(VX_SDA);
  Wire.setSCL(VX_SCL);
  Wire.begin();
  while(!lox.begin(VL53L0X_I2C_ADDR,false,&Wire));  //初始化VL53L0X对象，将IIC对象地址传入
}


void loop() {
  String command = Serial1.readStringUntil('\n');

  if(command == "ls") {
    Serial1.println("VL_msg,VRef,Temp,V_A0");
  }
  else if(command == "VL_msg"){
    VL53L0X_RangingMeasurementData_t measure;
    lox.rangingTest(&measure,false);
    if(measure.RangeStatus != 4) Serial1.println(measure.RangeMilliMeter);
    else Serial1.println("out of range");
  }
  else if(command == "VRef") {
    int32_t VRef = readVref();
    Serial1.printf("VRef(mv)= %i\n", VRef);
  }
  else if(command == "V_A0"){
    int32_t VRef = readVref();
    Serial1.printf("V_A0= %i\n", readVoltage(VRef, A0));
  }
  else if(command == "Temp"){
    int32_t VRef = readVref();
    Serial1.printf("Temp(°C)= %i\n", readTempSensor(VRef));
  }
  else if(command == "") return;
  else 
    Serial1.println("unknown command");
}

#endif
