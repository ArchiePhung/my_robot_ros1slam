/*
 * Author: Automatic Addison
 * Website: https://automaticaddison.com
 * Description: ROS node that publishes the accumulated ticks for each wheel
 * (/right_ticks and /left_ticks topics) at regular intervals using the 
 * built-in encoder (forward = positive; reverse = negative). 
 * The node also subscribes to linear & angular velocity commands published on 
 * the /cmd_vel topic to drive the robot accordingly.
 * Reference: Practical Robotics in C++ book (ISBN-10 : 9389423465)
 */
#include <Arduino.h>
#include <ros.h>
#include <std_msgs/Int16.h>
// #include <std_msgs/Int16MultiArray.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float64.h>

// Handles startup and shutdown of ROS
ros::NodeHandle nh;
 
////////////////// Tick Data Publishing Variables and Constants ///////////////
 
// Encoder output to Arduino Interrupt pin. Tracks the tick count.
#define ENC_IN_LEFT_A 3
#define ENC_IN_RIGHT_A 2
 
// Other encoder output to Arduino to keep track of wheel direction
// Tracks the direction of rotation.
#define ENC_IN_LEFT_B 5
#define ENC_IN_RIGHT_B 4
 
// True = Forward; False = Reverse
bool Direction_left = true;
bool Direction_right = true;
 
// Minumum and maximum values for 16-bit integers
// Range of 65,535
const int encoder_minimum = -32768;
const int encoder_maximum = 32767;
 
// Keep track of the number of wheel ticks
std_msgs::Int16 right_wheel_tick_count;
ros::Publisher rightPub("right_ticks", &right_wheel_tick_count);
 
std_msgs::Int16 left_wheel_tick_count;
ros::Publisher leftPub("left_ticks", &left_wheel_tick_count);

std_msgs::Float64 left_wheel_vel;
ros::Publisher leftVelPub("left_wheel_vel", &left_wheel_vel);

std_msgs::Float64 right_wheel_vel;
ros::Publisher rightVelPub("right_wheel_vel", &right_wheel_vel);

geometry_msgs::Twist core_data;
ros::Publisher CoreData("core_info", &core_data);

// Time interval for measurements in milliseconds
const int interval = 30;
long previousMillis = 0;
long currentMillis = 0;
 
////////////////// Motor Controller Variables and Constants ///////////////////
 
// Motor A connections -- left
// const int enA = 11;
const int in1 = 11;
const int in2 = 6;
  
// Motor B connections -- right
// const int enB = 6; 
const int in3 = 10;
const int in4 = 9;
 
// How much the PWM value can change each cycle
const int PWM_INCREMENT = 1;
 
// Number of ticks per wheel revolution. We won't use this in this code.
const int TICKS_PER_REVOLUTION = 858; //620: origin - 800: for left wheel real - 810: for right wheel real
 
// Wheel radius in meters
const double WHEEL_RADIUS = 0.0325;
 
// Distance from center of the left tire to the center of the right tire in m
const double WHEEL_BASE = 0.27;
 
// Number of ticks a wheel makes moving a linear distance of 1 meter
// This value was measured manually.
const double TICKS_PER_METER = 4200; // Originally 2880
 
// Proportional constant, which was measured by measuring the 
// PWM-Linear Velocity relationship for the robot.
const int K_P = 278;
 
// Y-intercept for the PWM-Linear Velocity relationship for the robot
const int b = 52;
 
// Correction multiplier for drift. Chosen through experimentation.
const int DRIFT_MULTIPLIER = 120;
 
// Turning PWM output (0 = min, 255 = max for PWM values)
const int PWM_TURN = 80;
 
// Set maximum and minimum limits for the PWM values
const int PWM_MIN = 0; // about 0.1 m/s
const int PWM_MAX = 255; // about 0.172 m/s
 
// Set linear velocity and PWM variable values for each wheel
double velLeftWheel = 0;
double velRightWheel = 0;
double pwmLeftReq = 0;
double pwmRightReq = 0;

double vel_left = 0;
double vel_right = 0;
// Record the time that the last velocity command was received
double lastCmdVelReceived = 0;

// direction of robot 
int leftwheel_dir = 0;
int rightwheel_dir = 0;

int is_recv_left_wheel = 0;
int is_recv_right_wheel = 0;
int robot_dir = 0;
long prevT = 0;
float eintegral_r = 0;
float eintegral_l = 0;
int pwmout1 = 0;
int pwmout2 = 0;
int pre_robot_dir = 0;
/////////////////////// Tick Data Publishing Functions ////////////////////////
 
// Increment the number of ticks
void right_wheel_tick() {
   
  // Read the value for the encoder for the right wheel
  int val = digitalRead(ENC_IN_RIGHT_B);
 
  if (val == LOW) {
    Direction_right = true; // Forward
  }
  else {
    Direction_right = false; // Reverse
  }
   
  if (Direction_right) {
     
    if (right_wheel_tick_count.data == encoder_maximum) {
      right_wheel_tick_count.data = encoder_minimum;
    }
    else {
      right_wheel_tick_count.data++;  
    }    
  }
  else {
    if (right_wheel_tick_count.data == encoder_minimum) {
      right_wheel_tick_count.data = encoder_maximum;
    }
    else {
      right_wheel_tick_count.data--;  
    }   
  }
}
 
// Increment the number of ticks
void left_wheel_tick() {
   
  // Read the value for the encoder for the left wheel
  int val = digitalRead(ENC_IN_LEFT_B);
 
  if (val == LOW) {
    Direction_left = false; // Forward
  }
  else {
    Direction_left = true; // Reverse
  }
   
  if (Direction_left) {
    if (left_wheel_tick_count.data == encoder_maximum) {
      left_wheel_tick_count.data = encoder_minimum;
    }
    else {
      left_wheel_tick_count.data++;  
    }  
  }
  else {
    if (left_wheel_tick_count.data == encoder_minimum) {
      left_wheel_tick_count.data = encoder_maximum;
    }
    else {
      left_wheel_tick_count.data--;  
    }   
  }
}
 
/////////////////////// Motor Controller Functions ////////////////////////////
 
// Calculate the left wheel linear velocity in m/s every time a 
// tick count message is rpublished on the /left_ticks topic. 
void calc_vel_left_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevLeftCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 + left_wheel_tick_count.data - prevLeftCount) % 65535;
 
  // If we have had a big jump, it means the tick count has rolled over.
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per millisecond
  velLeftWheel = numOfTicks/TICKS_PER_METER/(millis()-prevTime);

  // Calculate right wheel velocity in RPM
  left_wheel_vel.data = velLeftWheel*30000.0/PI/WHEEL_RADIUS;

  // Keep track of the previous tick count
  prevLeftCount = left_wheel_tick_count.data;
 
  // Update the timestamp
  prevTime = millis();
 
}
 
// Calculate the right wheel linear velocity in m/s every time a 
// tick count message is published on the /right_ticks topic. 
void calc_vel_right_wheel(){
   
  // Previous timestamp
  static double prevTime = 0;
   
  // Variable gets created and initialized the first time a function is called.
  static int prevRightCount = 0;
 
  // Manage rollover and rollunder when we get outside the 16-bit integer range 
  int numOfTicks = (65535 + right_wheel_tick_count.data - prevRightCount) % 65535;
 
  if (numOfTicks > 10000) {
        numOfTicks = 0 - (65535 - numOfTicks);
  }
 
  // Calculate wheel velocity in meters per milisecond
  velRightWheel = numOfTicks/TICKS_PER_METER/(millis()-prevTime);

  // Calculate right wheel velocity in RPM
  right_wheel_vel.data = velRightWheel*30000.0/PI/WHEEL_RADIUS;
  // right_wheel_vel.data = millis() - prevTime;
 
  prevRightCount = right_wheel_tick_count.data;
   
  prevTime = millis();
 
}
 
// Take the velocity command as input and calculate the PWM values.
// void calc_pwm_values(const geometry_msgs::Twist& cmdVel) {
   
//   // Record timestamp of last velocity command received
//   lastCmdVelReceived = (millis()/1000);
   
//   // Calculate the PWM value given the desired velocity 
//   pwmLeftReq = K_P * cmdVel.linear.x + b;
//   pwmRightReq = K_P * cmdVel.linear.x + b;
 
//   // Check if we need to turn 
//   if (cmdVel.angular.z != 0.0) {
 
//     // Turn left
//     if (cmdVel.angular.z > 0.0) {
//       pwmLeftReq = -PWM_TURN;
//       pwmRightReq = PWM_TURN;
//     }
//     // Turn right    
//     else {
//       pwmLeftReq = PWM_TURN;
//       pwmRightReq = -PWM_TURN;
//     }
//   }
//   // Go straight
//   else {
     
//     // Remove any differences in wheel velocities 
//     // to make sure the robot goes straight
//     static double prevDiff = 0;
//     static double prevPrevDiff = 0;
//     double currDifference = velLeftWheel - velRightWheel; 
//     double avgDifference = (prevDiff+prevPrevDiff+currDifference)/3;
//     prevPrevDiff = prevDiff;
//     prevDiff = currDifference;
 
//     // Correct PWM values of both wheels to make the vehicle go straight
//     pwmLeftReq -= (int)(avgDifference * DRIFT_MULTIPLIER);
//     pwmRightReq += (int)(avgDifference * DRIFT_MULTIPLIER);
//   }
 
//   // Handle low PWM values
//   if (abs(pwmLeftReq) < PWM_MIN) {
//     pwmLeftReq = 0;
//   }
//   if (abs(pwmRightReq) < PWM_MIN) {
//     pwmRightReq = 0;  
//   }  
// }

void calc_left_wheel_query(const std_msgs::Int16& vel){
    is_recv_left_wheel = 1;
    vel_left = vel.data;
    lastCmdVelReceived = (millis()/1000);
    // if (abs(pwmLeftReq) < PWM_MIN) {
    //   pwmLeftReq = 0;
    // }
}

void calc_right_wheel_query(const std_msgs::Int16& vel){
    vel_right = vel.data;
    is_recv_right_wheel = 1;
    lastCmdVelReceived = (millis()/1000);
    // if (abs(pwmRightReq) < PWM_MIN) {
    //   pwmRightReq = 0;  
    // }

}

int gain_dir(int x, int y){
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

// void set_pwm_values() {
 
//   // These variables will hold our desired PWM values
//   static int pwmLeftOut = 0;
//   static int pwmRightOut = 0;
 
//   // If the required PWM is of opposite sign as the output PWM, we want to
//   // stop the car before switching direction
//   // static bool stopped = false;
//   if ((pwmLeftReq * velLeftWheel < 0 && pwmLeftOut != 0) ||
//       (pwmRightReq * velRightWheel < 0 && pwmRightOut != 0)) {
//     pwmLeftReq = 0;
//     pwmRightReq = 0;
//   }
 
//   // Set the direction of the motors
//   if (pwmLeftReq > 0) { // Left wheel forward
//     // digitalWrite(in1, HIGH);
//     // digitalWrite(in2, LOW);
//     leftwheel_dir = 1;
//   }
//   else if (pwmLeftReq < 0) { // Left wheel reverse
//     // digitalWrite(in1, LOW);
//     // digitalWrite(in2, HIGH);
//     leftwheel_dir = -1;
//   }
//   else if (pwmLeftReq == 0 && pwmLeftOut == 0 ) { // Left wheel stop
//     // digitalWrite(in1, LOW);
//     // digitalWrite(in2, LOW);
//     leftwheel_dir = 0;
//   }
//   else { // Left wheel stop
//     // digitalWrite(in1, LOW);
//     leftwheel_dir = 0;
//     // digitalWrite(in2, LOW); 
//   }
 
//   if (pwmRightReq > 0) { // Right wheel forward
//     // digitalWrite(in3, HIGH);
//     // digitalWrite(in4, LOW);
//     rightwheel_dir = 1;
//   }
//   else if(pwmRightReq < 0) { // Right wheel reverse
//     // digitalWrite(in3, LOW);
//     // digitalWrite(in4, HIGH);
//     rightwheel_dir = -1;
//   }
//   else if (pwmRightReq == 0 && pwmRightOut == 0) { // Right wheel stop
//     // digitalWrite(in3, LOW);
//     // digitalWrite(in4, LOW);
//     rightwheel_dir = 0;
//   }
//   else { // Right wheel stop
//     // digitalWrite(in3, LOW);
//     // digitalWrite(in4, LOW);
//     rightwheel_dir = 0;
//   }
 
//   // Increase the required PWM if the robot is not moving
//   if (pwmLeftReq != 0 && velLeftWheel == 0) {
//     pwmLeftReq *= 1.5;
//   }
//   if (pwmRightReq != 0 && velRightWheel == 0) {
//     pwmRightReq *= 1.5;
//   }
 
//   // Calculate the output PWM value by making slow changes to the current value
//   if (abs(pwmLeftReq) > pwmLeftOut) {
//     pwmLeftOut += PWM_INCREMENT;
//   }
//   else if (abs(pwmLeftReq) < pwmLeftOut) {
//     pwmLeftOut -= PWM_INCREMENT;
//   }
//   else{}
   
//   if (abs(pwmRightReq) > pwmRightOut) {
//     pwmRightOut += PWM_INCREMENT;
//   }
//   else if(abs(pwmRightReq) < pwmRightOut) {
//     pwmRightOut -= PWM_INCREMENT;
//   }
//   else{}
 
//   // Conditional operator to limit PWM output at the maximum 
//   pwmLeftOut = (pwmLeftOut > PWM_MAX) ? PWM_MAX : pwmLeftOut;
//   pwmRightOut = (pwmRightOut > PWM_MAX) ? PWM_MAX : pwmRightOut;
 
//   // PWM output cannot be less than 0
//   pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
//   pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;
 
//   // Set the PWM value on the pins
//   if(leftwheel_dir == 1){
//     analogWrite(in1, pwmLeftOut);
//     analogWrite(in2, 0);
//   }
//   else if(leftwheel_dir == -1){
//     analogWrite(in1, 0);
//     analogWrite(in2, pwmLeftOut);
//   }
//   else{
//     analogWrite(in1, 0);
//     analogWrite(in2, 0);
//   }

//   if(rightwheel_dir == 1){
//     analogWrite(in3, pwmRightOut);
//     analogWrite(in4, 0);
//   }
//   else if(rightwheel_dir == -1){
//     analogWrite(in3, 0);
//     analogWrite(in4, pwmRightOut);
//   }
//   else{
//     analogWrite(in3, 0);
//     analogWrite(in4, 0);
//   }

//   // analogWrite(enA, pwmLeftOut); 
//   // analogWrite(enB, pwmRightOut); 
// }

void set_pwm_values(int dir, double pwm_left, double pwm_right) {
 
  // These variables will hold our desired PWM values
  static int pwmLeftOut = 0;
  static int pwmRightOut = 0;
 
  // If the required PWM is of opposite sign as the output PWM, we want to
  // stop the car before switching direction
  // static bool stopped = false;
  // if ((pwmLeftReq * velLeftWheel < 0 && pwmLeftOut != 0) ||
  //     (pwmRightReq * velRightWheel < 0 && pwmRightOut != 0)) {
  //   pwmLeftReq = 0;
  //   pwmRightReq = 0;
  // }
 
  // Increase the required PWM if the robot is not moving
  // if (pwmLeftReq != 0 && velLeftWheel == 0) {
  //   pwmLeftReq *= 1.5;
  // }
  // if (pwmRightReq != 0 && velRightWheel == 0) {
  //   pwmRightReq *= 1.5;
  // }
 
  // Calculate the output PWM value by making slow changes to the current value
  if (abs(pwmLeftReq) > pwmLeftOut) {
    pwmLeftOut += PWM_INCREMENT;
  }
  else if (abs(pwmLeftReq) < pwmLeftOut) {
    pwmLeftOut -= PWM_INCREMENT;
  }
  else{}
   
  if (abs(pwmRightReq) > pwmRightOut) {
    pwmRightOut += PWM_INCREMENT;
  }
  else if(abs(pwmRightReq) < pwmRightOut) {
    pwmRightOut -= PWM_INCREMENT;
  }
  else{}
 
  // Conditional operator to limit PWM output at the maximum 
  pwmLeftOut = (pwmLeftOut > PWM_MAX) ? PWM_MAX : pwmLeftOut;
  pwmRightOut = (pwmRightOut > PWM_MAX) ? PWM_MAX : pwmRightOut;
 
  // PWM output cannot be less than 0
  pwmLeftOut = (pwmLeftOut < 0) ? 0 : pwmLeftOut;
  pwmRightOut = (pwmRightOut < 0) ? 0 : pwmRightOut;

  pwmout1 = pwmLeftOut;
  pwmout2 = pwmRightOut;
 
  // Set the PWM value on the pins
  if(dir == 1){
    analogWrite(in1, pwmLeftOut);
    analogWrite(in2, 0);
    analogWrite(in3, pwmRightOut);
    analogWrite(in4, 0);
  }
  else if(dir == 2){      // phair
    analogWrite(in1, 0);
    analogWrite(in2, pwmLeftOut);
    analogWrite(in3, pwmRightOut);
    analogWrite(in4, 0);
  }
  else if(dir == 3){      // trai
    analogWrite(in1, pwmLeftOut);
    analogWrite(in2, 0);
    analogWrite(in3, 0);
    analogWrite(in4, pwmRightOut);
  }

  else if(dir == 4){      // lui
    analogWrite(in1, 0);
    analogWrite(in2, pwmLeftOut);
    analogWrite(in3, 0);
    analogWrite(in4, pwmRightOut);
  }

  else{
    analogWrite(in1, 0);
    analogWrite(in2, 0);
    analogWrite(in3, 0);
    analogWrite(in4, 0);
  }
}


// Set up ROS subscriber to the velocity command
// ros::Subscriber<geometry_msgs::Twist> subCmdVel("cmd_vel", &calc_pwm_values );

ros::Subscriber<std_msgs::Int16> left_wheel_query("left_wheel_query", &calc_left_wheel_query );
ros::Subscriber<std_msgs::Int16> right_wheel_query("right_wheel_query", &calc_right_wheel_query );

void setup() {
  // Set pin states of the encoder
  pinMode(ENC_IN_LEFT_A , INPUT_PULLUP);
  pinMode(ENC_IN_LEFT_B , INPUT);
  pinMode(ENC_IN_RIGHT_A , INPUT_PULLUP);
  pinMode(ENC_IN_RIGHT_B , INPUT);
 
  // Every time the pin goes high, this is a tick
  attachInterrupt(digitalPinToInterrupt(ENC_IN_LEFT_A), left_wheel_tick, RISING);
  attachInterrupt(digitalPinToInterrupt(ENC_IN_RIGHT_A), right_wheel_tick, RISING);
   
  // Motor control pins are outputs
  // pinMode(enA, OUTPUT);
  // pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  
  // Turn off motors - Initial state
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
  
  // Set the motor speed
  // analogWrite(enA, 0); 
  // analogWrite(enB, 0);
 
  // ROS Setup
  nh.getHardware()->setBaud(57600);
  nh.initNode();
  nh.advertise(rightPub);
  nh.advertise(leftPub);
  nh.advertise(leftVelPub);
  nh.advertise(rightVelPub);
  // nh.advertise(DirData);
  nh.advertise(CoreData);
  nh.subscribe(left_wheel_query);
  nh.subscribe(right_wheel_query);
}
 
void loop() {
   
  nh.spinOnce();
   
  // Record the time
  currentMillis = millis();
 
  // If the time interval has passed, publish the number of ticks,
  // and calculate the velocities.
  if (currentMillis - previousMillis > interval) {
     
    previousMillis = currentMillis;
 
    // Publish tick counts to topics
    leftPub.publish( &left_wheel_tick_count );
    rightPub.publish( &right_wheel_tick_count );
 
    // Calculate the velocity of the right and left wheels
    calc_vel_right_wheel();
    calc_vel_left_wheel();

    leftVelPub.publish(&left_wheel_vel);
    rightVelPub.publish(&right_wheel_vel);
     
  }

  if (is_recv_left_wheel == 1 && is_recv_right_wheel == 1){

    // Compute velocity with method 1
    long currT = micros();
    float deltaT = ((float) (currT-prevT))/1.0e6;

    // -- ROBOT RUN MANUALLY --
    // -- set vel target for left motor -- 
    float vtl = abs(vel_left);
    float kpl = 10;
    float kil = 0;
    float el = vtl - abs(left_wheel_vel.data);
    eintegral_l = eintegral_l + el*deltaT;

    float ul = kpl*el + kil*eintegral_l;

    // u = constrain(u, PWM_MIN_RIGHT_MOTOR, PWM_MAX_RIGHT_MOTOR);

    pwmLeftReq = fabs(ul);
    if(pwmLeftReq > PWM_MAX){
      pwmLeftReq = PWM_MAX;
    }

    // -- set vel target for right motor -- 
    float vtr = abs(vel_right);
    float kpr = 10;
    float kir = 0;
    float er = vtr - abs(right_wheel_vel.data);
    eintegral_r = eintegral_r + er*deltaT;

    float ur = kpr*er + kir*eintegral_r;

    // u = constrain(u, PWM_MIN_RIGHT_MOTOR, PWM_MAX_RIGHT_MOTOR);

    pwmRightReq = fabs(ur);
    if(pwmRightReq > PWM_MAX){
      pwmRightReq = PWM_MAX;
    }

    // -- robot run -- 
    // determine direction of robot
    robot_dir = gain_dir(vel_left, vel_right);

    // Case 1: robot stop => reset variables
    if (robot_dir == 0) {
      ul = 0;
      pwmLeftReq = 0;
      eintegral_l = 0;

      ur = 0;
      pwmRightReq = 0;
      eintegral_r = 0;
      
    }

    // Case 2: robot change dir => stop robot before running
    if (pre_robot_dir != robot_dir){
      pre_robot_dir = robot_dir;
      ul = 0;
      pwmLeftReq = 0;
      eintegral_l = 0;

      ur = 0;
      pwmRightReq = 0;
      eintegral_r = 0;      
    }
    
    // Stop the car if there are no cmd_vel messages
    if((millis()/1000) - lastCmdVelReceived > 1) {
      pwmLeftReq = 0;
      pwmRightReq = 0;
    }

    set_pwm_values(robot_dir, pwmLeftReq, pwmRightReq);

    core_data.linear.x = pwmLeftReq;
    core_data.linear.y = pwmRightReq;
    core_data.angular.x = pwmout1;
    core_data.angular.y = pwmout2;
    core_data.linear.z = robot_dir;

    CoreData.publish(&core_data);

  }
}