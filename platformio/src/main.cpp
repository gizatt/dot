/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"

#include <ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/String.h>

ros::NodeHandle nh;

std_msgs::String str_msg;
ros::Publisher chatter("chatter", &str_msg);

const int STR_LEN = 1000;
char hello[STR_LEN];

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

class CurrentSensor {
    public:
        const int A2D_CURRENT_PIN = 24;
        const double MAX_CURRENT = 50; // Amps
        const double MAX_CURRENT_RAW = 1024; // AnalogRead output unit
        const double UPDATE_PERIOD = 0.00000001;
        CurrentSensor(ros::NodeHandle& nh) :
                m_pub("speck/current", &m_float_msg){
            m_last_update_t = nh.now();
            nh.advertise(m_pub);
        }

        void update(ros::Time t){
            if (t.toSec() - m_last_update_t.toSec() > UPDATE_PERIOD){
                m_last_update_t = t;
                //double val = (double)analogRead(A2D_CURRENT_PIN);
                double val = 0.;
                m_float_msg.data = MAX_CURRENT * (val /  MAX_CURRENT_RAW);
                m_pub.publish( &m_float_msg );
            }
        }
    private:
        std_msgs::Float32 m_float_msg;
        ros::Publisher m_pub;
        ros::Time m_last_update_t;
};

class LEDBlinker {
    public:
        double blink_period = 0.5;
        LEDBlinker(ros::NodeHandle& nh, int led_pin) : m_led_pin(led_pin){
            m_last_update_t = nh.now();
            m_led_state = false;
            update_led();
        }

        void update_led(){
            if (m_led_state){
                digitalWrite(m_led_pin, HIGH);
            } else {
                digitalWrite(m_led_pin, LOW);
            }
        }

        void update(ros::Time t){
            if (t.toSec() - m_last_update_t.toSec() > blink_period){
                m_last_update_t = t;
                m_led_state = !m_led_state;
                update_led();
            }
        }
    private:
        int m_led_pin;
        bool m_led_state;
        ros::Time m_last_update_t;
};

CurrentSensor * currentSensor;
LEDBlinker * blinker;
void setup()
{
  // initialize LED digital pin as an output.
  pinMode(LED_BUILTIN, OUTPUT);
  
  nh.getHardware()->setBaud(9800);
  nh.initNode();
  nh.advertise(chatter);

  blinker = new LEDBlinker(nh, LED_BUILTIN);
  currentSensor = new CurrentSensor(nh);

  for (int i = 0; i < STR_LEN; i++){
      hello[i] = 'A' + (i%25);
  }
  hello[STR_LEN-1] = 0;
}

void loop()
{
  auto t = nh.now();

  //currentSensor->update(t);
  blinker->update(t);

  str_msg.data = hello;
  chatter.publish(&str_msg);
  
  nh.spinOnce();
}