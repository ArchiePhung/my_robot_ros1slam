#include <Arduino.h>
#include "ros.h"
#include <message_pkg/Vel_msg.h>
#include <geometry_msgs/Twist.h>
#include "param_config.h"
#include "pin_config.h"
#include "main.h"
#include <util/atomic.h>
#include <std_msgs/Int32.h>

// variables
int w_r=0, w_l=0;
double speed_ang=0, speed_lin=0;
double wheel_rad = 0.0325, wheel_sep = 0.295;

long tim = 0;

message_pkg::Vel_msg vel_info;

ESP32_Controller* Main_ctrl;
Robot myrobot;

// globals
long prevT = 0;
int posPrev_left = 0;
int posPrev_right = 0;

// Use the "volatile" directive for variables
// used in an interrupt
volatile long pos_left = 0;
volatile long pos_right = 0;
volatile float velocity_i = 0;
volatile long prevT_i = 0;

float vFilt_left = 0;
float vPrev_left = 0;
float vFilt_right = 0;
float vPrev_right = 0;

float eintegral_r = 0;
float eintegral_l = 0;

// time pub data 
long currentMillis = 0;
const int interval = 30;
long previousMillis = 0;

void calc_left_wheel_query(const std_msgs::Int16& vel){
    is_recv_left_wheel = 1;
    vel_left = vel.data;
    lastCmdVelReceived = (millis()/1000);

}

void calc_right_wheel_query(const std_msgs::Int16& vel){
    vel_right = vel.data;
    is_recv_right_wheel = 1;
    lastCmdVelReceived = (millis()/1000);

}

ros::NodeHandle nh;
// ros::Subscriber<geometry_msgs::Twist> sub("/cmd_vel", &messageCb);
ros::Publisher pwm_info("/pwm_info", &vel_info);

ros::Subscriber<std_msgs::Int16> left_wheel_query("left_wheel_query", &calc_left_wheel_query );
ros::Subscriber<std_msgs::Int16> right_wheel_query("right_wheel_query", &calc_right_wheel_query );

std_msgs::Int32 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int32 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

void ESP32_Controller::pin_init()
{
   pinMode(LEFT_MOTOR_PIN_H, OUTPUT);
   pinMode(LEFT_MOTOR_PIN_L, OUTPUT);
   pinMode(LEFT_ENABLE_MOTOR, OUTPUT);
   pinMode(RIGHT_MOTOR_PIN_H, OUTPUT);
   pinMode(RIGHT_MOTOR_PIN_L, OUTPUT);
   pinMode(RIGHT_ENABLE_MOTOR, OUTPUT);

   pinMode(LEFT_ENCA, INPUT);
   pinMode(LEFT_ENCB, INPUT);
   pinMode(RIGHT_ENCA, INPUT);
   pinMode(RIGHT_ENCA, INPUT);
}

void Robot::stop(){
   digitalWrite(LEFT_MOTOR_PIN_H, 0);
   digitalWrite(LEFT_MOTOR_PIN_L, 0);
   analogWrite(LEFT_ENABLE_MOTOR, 0);
   digitalWrite(RIGHT_MOTOR_PIN_H, 0);
   digitalWrite(RIGHT_MOTOR_PIN_L, 0);
   analogWrite(RIGHT_ENABLE_MOTOR, 0);
}

void Robot::run(int dir_, int Motor_speeda, int Motor_speedb)
{
   if( dir_ == 1){
      run_forward();
      analogWrite(LEFT_ENABLE_MOTOR, abs(Motor_speeda));
      analogWrite(RIGHT_ENABLE_MOTOR, abs(Motor_speedb));
   }

   else if( dir_ == 2){
      turn_left();
      analogWrite(LEFT_ENABLE_MOTOR, abs(int(0.6*Motor_speeda)));
      analogWrite(RIGHT_ENABLE_MOTOR, abs(int(0.6*Motor_speedb)));      
   }

   else if( dir_ == 3){
      turn_right();
      analogWrite(LEFT_ENABLE_MOTOR, abs(int(0.6*Motor_speeda)));
      analogWrite(RIGHT_ENABLE_MOTOR, abs(int(0.6*Motor_speedb)));           
   }

   else if( dir_ == 4){
      run_backward();
      analogWrite(LEFT_ENABLE_MOTOR, abs(Motor_speeda));
      analogWrite(RIGHT_ENABLE_MOTOR, abs(Motor_speedb));      
   }
   else{
      stop();
      analogWrite(LEFT_ENABLE_MOTOR, 0);
      analogWrite(RIGHT_ENABLE_MOTOR, 0);
   }
}

int Robot::gain_dir(int x, int y){
   if( x > 0 && y > 0){
      return 1;                                   // tiến trước 
   }

   else if(( x > 0 && y < 0) || (x > 0 && y == 0) || (x == 0 && y < 0)){
      return 2;                                   // quay phải 
   }

   else if(( x < 0 && y > 0) || (x == 0 && y > 0) || (x < 0 && y == 0)){
      return 3;                                  // quay trái 
   }

   else if(x < 0 && y < 0 ){
      return 4;                                   // lùi sau
   }

   else{                                         // đứng yên
      return 0;
   }
}

void Robot::run_forward(){
   digitalWrite(LEFT_MOTOR_PIN_H, 1);
   digitalWrite(LEFT_MOTOR_PIN_L, 0);
   digitalWrite(RIGHT_MOTOR_PIN_H, 1);
   digitalWrite(RIGHT_MOTOR_PIN_L, 0);
}

void Robot::run_backward(){
   digitalWrite(LEFT_MOTOR_PIN_H, 0);
   digitalWrite(LEFT_MOTOR_PIN_L, 1);
   digitalWrite(RIGHT_MOTOR_PIN_H, 0);
   digitalWrite(RIGHT_MOTOR_PIN_L, 1);
}

void Robot::turn_right(){
   digitalWrite(LEFT_MOTOR_PIN_H, 0);
   digitalWrite(LEFT_MOTOR_PIN_L, 1);
   digitalWrite(RIGHT_MOTOR_PIN_H, 1);
   digitalWrite(RIGHT_MOTOR_PIN_L, 0);
}

void Robot::turn_left(){
   digitalWrite(LEFT_MOTOR_PIN_H, 1);
   digitalWrite(LEFT_MOTOR_PIN_L, 0);
   digitalWrite(RIGHT_MOTOR_PIN_H, 0);
   digitalWrite(RIGHT_MOTOR_PIN_L, 1);
}

void ESP32_Controller::launch_init()
{
   // ledcSetup(0,1024,10);
   // ledcSetup(1,1024,10);
   // ledcSetup(2,1024,10);
   // ledcSetup(3,1024,10);

   // ledcAttachPin(LEFT_MOTOR_PIN_H,0);
   // ledcAttachPin(LEFT_MOTOR_PIN_L,1);
   // ledcAttachPin(RIGHT_MOTOR_PIN_H,2);
   // ledcAttachPin(RIGHT_MOTOR_PIN_L,3);
   rb_ctrl->stop();
}

void readEncoder_left(){
   // Read encoder B when ENCA rises
   int b = digitalRead(LEFT_ENCB);
   int increment = 0;
   if(b>0){
      // If B is high, increment forward
      increment = 1;
   }
   else{
      // Otherwise, increment backward
      increment = -1;
   }
   pos_left = pos_left + increment;
   left_wheel_tick_count.data = pos_left;
}

void readEncoder_right(){
   // Read encoder B when ENCA rises
   int b = digitalRead(RIGHT_ENCB);
   int increment = 0;
   if(b>0){
      // If B is high, increment forward
      increment = -1;
   }
   else{
      // Otherwise, increment backward
      increment = +1;
   }
   pos_right = pos_right + increment;
   right_wheel_tick_count.data = pos_right;
   // Serial.println(pos_right);
}

void setup()
{  
   Main_ctrl->pin_init();
   Main_ctrl->launch_init();

   nh.initNode();
   nh.getHardware()->setBaud(57600);
   nh.subscribe(sub);
   nh.advertise(pwm_info);
   nh.advertise(rightPub);
   nh.advertise(leftPub);

   attachInterrupt(digitalPinToInterrupt(LEFT_ENCA) ,readEncoder_left, RISING);
   attachInterrupt(digitalPinToInterrupt(RIGHT_ENCA) ,readEncoder_right, RISING);

   tim = millis();
}

void loop()
{  
   // int pwr = 200;
   // myrobot.run_forward(pwr - 100, pwr + 30);

   // -- CALCULATE VELOCITY OF 2 WHEEL -- 
   long posleft = 0;
   long posright = 0;
   ATOMIC_BLOCK(ATOMIC_RESTORESTATE){
      posleft = pos_left;
      // left_wheel_tick_count.data = posleft;
      posright = pos_right;
      // right_wheel_tick_count.data = posright;
   }

   // Compute velocity with method 1
   long currT = micros();
   float deltaT = ((float) (currT-prevT))/1.0e6;

   float velocity_left = (posleft - posPrev_left)/deltaT;
   posPrev_left = posleft;

   float velocity_right = (posright - posPrev_right)/deltaT;
   posPrev_right = posright;

   prevT = currT;

   float v_left = velocity_left/(ENCODER_PULSE*MOTOR_RATIO)*60.0;
   float v_right = velocity_right/(ENCODER_PULSE*MOTOR_RATIO)*60.0;

   vFilt_left = 0.854*vFilt_left + 0.0728*v_left + 0.0728*vPrev_left;
   vPrev_left = v_left;

   vFilt_right = 0.854*vFilt_right + 0.0728*v_right + 0.0728*vPrev_right;
   vPrev_right = v_right;

   // Serial.print(vFilt_left);
   // Serial.print(" ");
   // Serial.println(vFilt_right);

   currentMillis = millis();
   
   // If the time interval has passed, publish the number of ticks,
   // and calculate the velocities.
   if (currentMillis - previousMillis > interval) {
      
      previousMillis = currentMillis;
   
      // Publish tick counts to topics
      leftPub.publish( &left_wheel_tick_count );
      rightPub.publish( &right_wheel_tick_count );
      
   }

   vel_info.vel_left_motor.data = vFilt_left;
   vel_info.vel_right_motor.data = vFilt_right;

   // -- ROBOT RUN MANUALLY --
   // -- set vel target for left motor -- 
   float vtl = TARGET_VEL;
   float kpl = 5;
   float kil = 10;
   float el = vtl - abs(vFilt_left);
   eintegral_l = eintegral_l + el*deltaT;
   
   float ul = kpl*el + kil*eintegral_l;

   // u = constrain(u, PWM_MIN_RIGHT_MOTOR, PWM_MAX_RIGHT_MOTOR);

   int pwr_left = (int) fabs(ul);
   if(pwr_left > PWM_MAX_LEFT_MOTOR){
      pwr_left = PWM_MAX_LEFT_MOTOR;
   }

   vel_info.pwm_left_motor.data = pwr_left;

   // -- set vel target for right motor -- 
   float vtr = TARGET_VEL;
   float kpr = 10;
   float kir = 20;
   float er = vtr - abs(vFilt_right);
   eintegral_r = eintegral_r + er*deltaT;
   
   float ur = kpr*er + kir*eintegral_r;

   // u = constrain(u, PWM_MIN_RIGHT_MOTOR, PWM_MAX_RIGHT_MOTOR);

   int pwr_right = (int) fabs(ur);
   if(pwr_right > PWM_MAX_RIGHT_MOTOR){
      pwr_right = PWM_MAX_RIGHT_MOTOR;
   }

   vel_info.pwm_right_motor.data = pwr_right;
   
   // -- robot run -- 
   myrobot.dir = myrobot.gain_dir(w_r, w_l);
   vel_info.dir.data = myrobot.dir;

   if (myrobot.dir == 0) {
      ul = 0;
      pwr_left = 0;
      eintegral_l = 0;

      ur = 0;
      pwr_right = 0;
      eintegral_r = 0;
      
   }

   myrobot.run(myrobot.dir, pwr_left, pwr_right);

   if(millis() - tim > 100){
      pwm_info.publish(&vel_info);
      tim = millis();
   }
   nh.spinOnce();
}

