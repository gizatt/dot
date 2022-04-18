/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"

#include "ros.h"
#include <std_msgs/Float32.h>
#include <std_msgs/Header.h>
#include <stdio.h>
#include <stdarg.h>

ros::NodeHandle nh;

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

class DebugStrPublisher {
    /*
     * Packs timestamped debug strs into a Header msg,
     * putting the debug str into the frame_id string field.
     */ 
    public:
        const static int BUF_LEN = 1000;
        DebugStrPublisher(ros::NodeHandle& nh) :
                m_nh(nh),
                m_pub("speck/debug", &m_msg){
            nh.advertise(m_pub);
        }
        template <typename ... Args>
        void log(Args&& ... args){
            snprintf(m_buf, BUF_LEN, args ...);
            m_msg.stamp = m_nh.now();
            m_msg.frame_id = m_buf;
            m_pub.publish( &m_msg );
        }

    private:
        ros::NodeHandle& m_nh;
        std_msgs::Header m_msg;
        ros::Publisher m_pub;
        char m_buf[BUF_LEN];
};
DebugStrPublisher * debugPublisher;

class CurrentSensor {
    public:
        const int A2D_CURRENT_PIN = 24;
        const double MAX_CURRENT = 50; // Amps
        const double MAX_CURRENT_RAW = 1024; // AnalogRead output unit
        const double UPDATE_PERIOD = 0.01;
        CurrentSensor(ros::NodeHandle& nh) :
                m_pub("speck/current", &m_float_msg){
            m_last_update_t = nh.now();
            nh.advertise(m_pub);
        }

        void update(ros::Time t){
            if (t.toSec() - m_last_update_t.toSec() > UPDATE_PERIOD){
                m_last_update_t = t;
                double val = (double)analogRead(A2D_CURRENT_PIN);
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

  blinker = new LEDBlinker(nh, LED_BUILTIN);
  currentSensor = new CurrentSensor(nh);
  debugPublisher = new DebugStrPublisher(nh);

  debugPublisher->log("Initialized.");
}

void loop()
{
  auto t = nh.now();

  currentSensor->update(t);
  blinker->update(t);
  nh.spinOnce();
}