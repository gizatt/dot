
#define USE_USBCON
#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/Int16MultiArray.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>


ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);
char send_buffer[256];

short int command_buffer[16];
void motorCommandCallback( const std_msgs::Int16MultiArray& command_msg){
  for (int i=0; i<min(command_msg.layout.dim[0].size, 16); i++){
    // WARNING: ignores offset and stride info in the command msg layout header.
    // Ignoring because I'll have a specialized message type in here soon.
    command_buffer[i] = command_msg.data[i];
  }
}
ros::Subscriber<std_msgs::Int16MultiArray> command_sub("motor_command", &motorCommandCallback );
const int MICROSECOND_MIN = 200;
const int MICROSECOND_MAX = 3000;
const int SERVO_UPDATE_PERIOD = 10; // Every 10 ms, send servo updates, if there are any.
long long last_servo_update_time;
short int sent_command_buffer[16];

Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
const int SERVO_OUTPUT_DISABLE_PIN = 19;

long long last_led_flip_time;
int led_state = 1;
void setup() {
  nh.initNode();
  Serial.begin(115200);
  nh.getHardware()->setBaud(115200);
  nh.advertise(chatter);
  nh.subscribe(command_sub);

  pwm.begin();
  pwm.setOscillatorFrequency(27000000);
  pwm.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
  pinMode(SERVO_OUTPUT_DISABLE_PIN, OUTPUT);
  digitalWrite(SERVO_OUTPUT_DISABLE_PIN, 0);
  for (int i = 0; i < 16; i++){
    command_buffer[i] = -1;
    sent_command_buffer[i] = -1;
  }
  last_servo_update_time = millis();

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

  // Update servo commands
  if (t < last_servo_update_time || t > last_servo_update_time + SERVO_UPDATE_PERIOD){
    for (int i = 0; i < 16; i++){
      if (command_buffer[i] != sent_command_buffer[i]){
        if (command_buffer[i] >= 0){
          if (command_buffer[i] >= MICROSECOND_MIN &&
              command_buffer[i] <= MICROSECOND_MAX){
            pwm.writeMicroseconds(i, command_buffer[i]); // commit the legal command
          } else {
            command_buffer[i] = -2; // error state
          }
        }
        if (command_buffer[i] < 0){
          pwm.setPWM(i, 0, 0); // Disable servo. Does this work?
        }
        sent_command_buffer[i] = command_buffer[i];
      }
    }
    last_servo_update_time = t;
  }


  int ptr = 0;
  for (int i = 0; i < 16; i++){
    ptr += sprintf(send_buffer + ptr, "%04d ", command_buffer[i]);
  }
  str_msg.data = send_buffer;
  chatter.publish( &str_msg );
  nh.spinOnce();

  delay(10);
}