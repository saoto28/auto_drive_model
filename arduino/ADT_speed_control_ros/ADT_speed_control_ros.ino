// Vehicle Driver for Autonoumous Driving Trailer model
// PCからシリアル通信により手動動作確認用
// - steering servo control
// - motor control
// - encoder input processing
// - angle sensor processing
//
// 2018/4/23 S.Tsuchiya
// 2018/5/9 ST: add speed control
// 2018/5/9 ST: add rosserial_arduino
// 2018/5/16 ST: steering control
// 2018/5/31 ST: modify trailer joint angle output deg -> rad

// for ROS
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/Twist.h>
//#include <nav_msgs/Odometry.h>

ros::NodeHandle node;
//std_msgs::String chat;
//std_msgs::Int16 Int1;
geometry_msgs::Twist velAct;
ros::Publisher pub("act_vel", &velAct);
//nav_msgs::Odometry odom;
//ros::Publisher pub("odom", &odom);

// for task timing
// need download Metro librarylibrary
// http://www.arduino.cc/playground/Code/Metro
#include <Metro.h>
Metro ledMetro1 = Metro(10);  //[ms] main task frequency

// for servo
#include <avr/io.h>
#define PIN_STR 9
#define PIN_MOT 10
#define MIN_SPD 120
unsigned int lambda_mot = 1000000/73; // [us]

// for analogin of angle sensor
const int analogInPin = A0;  // Analog input pin that the potentiometer is attached to

// int = int16
// long = int32
long spdReq;
long angReq;
int cnt_opration;
const int lengWhl = 290;    // [mm]
const double A_LengWhl = 1000.0/lengWhl;  // [1/m]
#define ANG_STR_MAX  500    // [mrad]
int flg_veh_drct = 1;  // 1 or -1
int sign(long val){return (val>0)-(val<0);}
// calback for ros topic //////////////////////////////////////////////////
void cmdCallback(const geometry_msgs::Twist &vel){
  cnt_opration = 0;
  long spdAng;
  
  spdReq = (long)(vel.linear.x *1000);    // m/s to mm/s
  spdAng = (long)(vel.angular.z * 1000);  // rad/s to mrad/s

// calc steering angle request
  if (spdReq == 0){
    angReq = 0;
  }else{
    angReq = (long)(spdAng * lengWhl / spdReq) ;  // [mrad]
    flg_veh_drct = sign(spdReq);  // vehicle direction
  }
  
  if(angReq < - ANG_STR_MAX){
    angReq = - ANG_STR_MAX;
  }else if(angReq > ANG_STR_MAX){
    angReq = ANG_STR_MAX;
  }
  
}

//ros::Subscriber<std_msgs::Bool> sub("led", &ledCallback);
ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel", &cmdCallback);

unsigned long cntPuls = 0;  // debug
// encoder //////////////////////////////////////////////////
volatile unsigned long periodEncoder = 60000000;
unsigned long microsLast = 0;
unsigned long tmpPeriod;
unsigned long tmpMicros; 
unsigned int counter = 0;

void timerEncoder(){
  tmpMicros = micros();
  tmpPeriod = tmpMicros - microsLast;
  if (tmpPeriod >= 300){    // neglect too short signal and overflow
    microsLast = tmpMicros;
    cntPuls++;
    periodEncoder = 0.8 * periodEncoder + 0.2 * tmpPeriod;
  }else{
  }
  //periodEncoder = tmpPeriod;
  counter ++;
  //periodEncoder = micros();   // debug

}

// setup //////////////////////////////////////////////////////
void setup() {
  Serial.begin(9600);     // opens serial port, sets data rate to 9600 bps
  while(!Serial);

  // rosserial_arduino
  node.initNode();
  node.subscribe(sub);
  node.advertise(pub);
  
  // initialize digital pin LED_BUILTIN as an output.
  pinMode(LED_BUILTIN, OUTPUT);

  // encoder
  //pinMode(2, INPUT);
  //attachInterrupt(digitalPinToInterrupt(2), timerEncoder, FALLING);  // error on ubuntu
  attachInterrupt(0, timerEncoder, FALLING);  // int0 means 2-pin for Uno
      
  pinMode(PIN_STR, OUTPUT);
  pinMode(PIN_MOT, OUTPUT);
}

// sirial communication //////////////////////////
char incomingByte = 0;   // for incoming serial data
int posUpDown = 0;
int posLeftRight = 0;
//////////////////////////////////////////////////
// not used for ROS
int readSerialCmd2(){
  static int cntKey = 0;    // 連続入力でしか機能しない
  int ret = 0;

  // send data only when you receive data:
  if (Serial.available() == 0) {
    cntKey = 0;
  }else{
    incomingByte = Serial.read();
//    if(incomingByte == 's'){
//      cntKey = 3;
//    }else if(cntKey == 3){
//      cntKey = 0;
      if('0' <= incomingByte && incomingByte <= '9'){
        spdReq = 100 * (incomingByte - '0');
        ret = 1;
//      }else if(incomingByte == '-'){
//        cntKey = 4;
//      }
    }else if(cntKey == 4){
      cntKey = 0;
      if('0' <= incomingByte && incomingByte <= '9'){
        spdReq = -100 * (incomingByte - '0');
        ret = 1;
      }
    }else if(incomingByte == 27){   // "ESC"
      cntKey = 1;
    }else if(cntKey == 1 && incomingByte == 91){  // "["
      cntKey = 2;
    }else if(cntKey == 2){
      cntKey = 0;

      if (incomingByte == 65){  // up
        if(-20<=posUpDown && posUpDown<40){
          posUpDown +=1;
        }else{
          posUpDown ++;
        }
      }else if(incomingByte == 66){  //down
        if(-20<posUpDown && posUpDown<=40){
          posUpDown -=1;
        }else{
          posUpDown --;
        }
      }else if(incomingByte == 67){  // right
        posLeftRight -= 1;
      }else if(incomingByte == 68){  // left
        posLeftRight += 1;
      }

      if(posLeftRight > 90){
        posLeftRight = 90;
      }else if(posLeftRight < -90){
        posLeftRight = -90;
      }

      if(posUpDown > 90){
        posUpDown = 90;
      }else if(posUpDown < -90){
        posUpDown = -90;
      }

      ret = 1;
    }else{
      cntKey = 0;
    } // check incomingByte
  } // Serial.available
  return ret;
}

// calculate speed //////////////////////////////////////////
// for tracter
#define RAT_GEAR (15.0/40.0)
#define NUM_HOLE 24    // the number of holes on the gear wheel
#define RAD_WHL 41   // [mm]
#define COEF_SPD (1000000.0 / NUM_HOLE * RAT_GEAR * 2.0 * 3.1415 * RAD_WHL)   //[mmps us]
int calcVehSpd(){
  int spd_veh;  // [mmps]
  
  if(counter == 0){   // no pulse @ 100ms
    spd_veh = 0;
  }else if(periodEncoder > COEF_SPD / 8){ // less than 8 mmps
    spd_veh = 0;
  }else{
    spd_veh = COEF_SPD / periodEncoder;
  }
  counter = 0;

  return spd_veh;
}

int err_intg = 0;
int flg_rev_inh = 1;
//////////////////////////////////////////////////
int speedControl(long spd_req, long spd_act){
  long spd_err;
  long cmd_ff, cmd_p, cmd_i;

  if(spd_req == 0){
    cmd_ff = 0;
    err_intg = 0;
    spd_err = 0;  // preliminary
    cmd_p = 0;  // preliminary
  }else if(spd_req > 0){
    flg_rev_inh = 1;                // リバース禁止フラグ
    
    spd_req = max(spd_req, 500);  // 暫定20180601ST
    cmd_ff = map(spd_req, 0, 4000, MIN_SPD, 240);
    spd_err = spd_req - spd_act;
    cmd_p = 0.04 * spd_err;
  }else if(spd_req < 0){
    flg_rev_inh = flg_rev_inh << 1; // リバース禁止時カウンター
    
    cmd_ff = map(spd_req, 0, -4000, -MIN_SPD, -240);
    spd_err = spd_req - spd_act;    // signed spd_act
    cmd_p = 0.04 * spd_err;
  }
  err_intg += spd_err;
  cmd_i = 0.02 * err_intg;
  cmd_i = min(400, cmd_i);
  cmd_i = max(-400, cmd_i);

  // リバース禁止解除処理
  if(flg_rev_inh > 7){
    flg_rev_inh = 0;
  }else if(flg_rev_inh > 3){
    return 0;
  }else if(flg_rev_inh > 1){
    return -200;
  }
  
  return cmd_ff + cmd_p + cmd_i;
}

long spd_veh_mmps;
long ang_joint;
int cmd_mot = 0;
// publish ROS topic ////////////////////////////
void pubActVel(){
//  chat.data = "comment";
  velAct.linear.x = (double)(spd_veh_mmps)/1000;
  velAct.linear.y = (double)(spdReq)/1000;
  velAct.linear.z = cmd_mot;
  velAct.angular.x = (double)(ang_joint)/1000*3.14/180.0;
  velAct.angular.y = (double)(angReq)/1000;
  velAct.angular.z = velAct.angular.y * A_LengWhl * velAct.linear.x;  // [rad/s]
  pub.publish(&velAct);
}

int sensor_value = 512;
long spd_veh_req = 0;
// Main Loop ////////////////////////////////////////////////////////////
void loop() {
  static int cnt_task1 = 5; // タイミングをずらす
  static int cnt_task2 = 0;
  
if (ledMetro1.check() == 1) {   // main task(10ms)

#if 1       // ROS
  node.spinOnce();

  if(cnt_opration > 50){    // stop output
    spdReq = 0;
    angReq = 0;
  }else{
    cnt_opration ++;
  }
  
  // http://blog.kts.jp.net/arduino-pwm-change-freq/
    // モード指定
    TCCR1A = 0b10100000; // <- COMA1を1にして9番ピンを有効化 + WGM10 を 0 にすることで ICR1 を TOP 値に指定
    TCCR1B = 0b00010010;
 
    // TOP値指定
    ICR1 = (unsigned int)(lambda_mot);  // PWM周期
    // Duty比指定
    OCR1A = (unsigned int)(1500 - angReq * 0.86); // steering servo PWM high time 
    spd_veh_req = spdReq;

#else       // Teraterm
  if(readSerialCmd2()){ // key input
    // http://blog.kts.jp.net/arduino-pwm-change-freq/
    // モード指定
    TCCR1A = 0b10100000; // <- COMA1を1にして9番ピンを有効化 + WGM10 を 0 にすることで ICR1 を TOP 値に指定
    TCCR1B = 0b00010010;
 
    // TOP値指定
    ICR1 = (unsigned int)(lambda_mot);  // PWM周期
    // Duty比指定
    OCR1A = (unsigned int)(1500 - posLeftRight * 10); // steering servo PWM high time 
 //   OCR1B = (unsigned int)(1500 - posUpDown * 5);     // motor PWM high time
    spd_veh_req = posUpDown * 100; // [mmps]
    
    Serial.print("                                         ");
    Serial.print("UpDown: ");
    Serial.print(posUpDown);
    Serial.print("\t");
    Serial.print("LeftRight: ");
    Serial.print(posLeftRight);
    Serial.println("");
  }
#endif

  cnt_task1 ++;
  if( cnt_task1 >10){   // 100ms
    cnt_task1 = 0;

    // speed sensor
    spd_veh_mmps = flg_veh_drct * calcVehSpd();
    cmd_mot = speedControl(spd_veh_req, spd_veh_mmps);
    OCR1B = (unsigned int)(1500 - cmd_mot);     // motor PWM high time
 
    // angle sensor
    sensor_value = analogRead(analogInPin);
    // map it to the range of the analog out:
    ang_joint = map(sensor_value, 176, 869, 90000, -90000);

//    chat.data = cmdMot;
//    pub.publish(&chat);
    pubActVel();
  }

  cnt_task2 ++;
  if (cnt_task2 >50){   // 500ms
    cnt_task2 = 0;
    digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));

#if 1
#else
    Serial.print("spd_req: ");
    Serial.print(spd_veh_req);
    Serial.print("\t spd_mmps: ");
    Serial.print(spd_veh_mmps);
    Serial.print("\t angle_joint_deg: ");
    Serial.print(ang_joint);
    Serial.println("");
#endif
  } // end of task2

} // end of Metro1
}


