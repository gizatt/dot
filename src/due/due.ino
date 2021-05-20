
#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>


ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

char hello[13] = "hello world!";

long long last_led_flip_time;
int led_state = 1;
void setup() {
  nh.initNode();
  nh.advertise(chatter);
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);

  last_led_flip_time = millis();
  pinMode(13, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  long long t = millis();
  if (t < last_led_flip_time || t > last_led_flip_time + 500){
    digitalWrite(13, led_state);   // turn the LED on (HIGH is the voltage level)
    led_state = !led_state;
    last_led_flip_time = t;
  }

  str_msg.data = hello;
  chatter.publish( &str_msg );
  nh.spinOnce();
}