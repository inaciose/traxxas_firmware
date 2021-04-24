// Author: sergio.inacio@ua.pt (inaciose@gmail.com)

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// remote ROS node required
// rosrun rosserial_python serial_node.py /dev/ttyUSB0
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

#if defined(ARDUINO) && ARDUINO >= 100
  #include "Arduino.h"
#else
  #include <WProgram.h>
#endif

#include <ros.h>
#include <std_msgs/Int16.h>

// PCA9685
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#define MIN_PULSE_WIDTH 600
#define MAX_PULSE_WIDTH 2600
#define FREQUENCY 50

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40);

ros::NodeHandle  nh;

int pulseWidth(int angle) {
  int pulse_wide, analog_value;
  pulse_wide = map(angle, 0, 180, MIN_PULSE_WIDTH, MAX_PULSE_WIDTH);
  analog_value = int(float(pulse_wide) / 1000000 * FREQUENCY * 4096);
  return analog_value;
}

void servo_dir( const std_msgs::Int16 & cmd_msg) {
  // steering is on channel 0
  pwm.setPWM(0, 0, pulseWidth(cmd_msg.data)); 
}

void servo_vel( const std_msgs::Int16 & cmd_msg) {
  // esc is on channel 1
  pwm.setPWM(1, 0, pulseWidth(cmd_msg.data)); 
}

ros::Subscriber<std_msgs::Int16> sub_dir("pub_dir", servo_dir);
ros::Subscriber<std_msgs::Int16> sub_vel("pub_vel", servo_vel);

void setup() {
  // init ros stuff
  nh.initNode();
  nh.subscribe(sub_dir);
  nh.subscribe(sub_vel);
  
  // init PCA9685 stuff
  pwm.begin();
  pwm.setPWMFreq(FREQUENCY);
}

void loop() {
  nh.spinOnce();
  delay(1);
} 


