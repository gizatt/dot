
#define USE_USBCON
#include <ros.h>
#include <dot_msgs/DueStatus.h>
#include <Wire.h>
#include "servo_set_controller.hpp"


ros::NodeHandle nh;
dot_msgs::DueStatus due_status_msg;
ros::Publisher chatter("due_status", &due_status_msg);
ServoSetController * servo_set_controller;

// Status message publish rate.
const double STATUS_MESSAGE_PERIOD = 0.5;
double last_status_message_time;

// Minimal state for flipping the Arduino LED, as visual proof-of-life.
double last_led_flip_time;
int led_state = 1;
void setup() {
  Serial.begin(250000);
  nh.getHardware()->setBaud(250000);
  nh.initNode();

  last_led_flip_time = nh.now().toSec() - 0.5;
  pinMode(13, OUTPUT);

  nh.advertise(chatter);
  last_status_message_time = nh.now().toSec() - 1.;
  servo_set_controller = new ServoSetController(nh);

  // Don't do anything else till we get a connection.
  while (!nh.connected()){
      nh.spinOnce();

      // Flash double-speed when waiting for connection.
      double t_double = nh.now().toSec();
      if (t_double < last_led_flip_time || t_double > last_led_flip_time + 0.25){
        digitalWrite(13, led_state);
        led_state = !led_state;
        last_led_flip_time = t_double;
      }
  }
}

// the loop function runs over and over again forever
void loop() {
  ros::Time t = nh.now();
  double t_double = t.toSec();
  if (t_double < last_led_flip_time || t_double > last_led_flip_time + 0.5){
    digitalWrite(13, led_state);
    led_state = !led_state;
    last_led_flip_time = t_double;
  }

  servo_set_controller->update(t);

  nh.spinOnce();
  // Periodically publish a status message.
  if (t_double < last_status_message_time || t_double > last_status_message_time + STATUS_MESSAGE_PERIOD) {
    due_status_msg.header.stamp.sec = t.sec;
    due_status_msg.header.stamp.nsec = t.nsec;
    ros::Time last_update_time = servo_set_controller->get_last_command_time();
    due_status_msg.average_update_rate = servo_set_controller->get_average_update_rate();
    due_status_msg.last_command = servo_set_controller->get_last_command();
    chatter.publish( &due_status_msg );
    
    last_status_message_time = t_double;
  }
  
  nh.spinOnce();
}