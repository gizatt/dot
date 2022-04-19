#pragma once

#include "ros.h"

class LEDBlinker
{
public:
    double blink_period = 0.5;
    LEDBlinker(ros::NodeHandle &nh, int led_pin) : m_led_pin(led_pin)
    {
        m_last_update_t = nh.now();
        m_led_state = false;
        update_led();
    }

    void update_led()
    {
        if (m_led_state)
        {
            digitalWrite(m_led_pin, HIGH);
        }
        else
        {
            digitalWrite(m_led_pin, LOW);
        }
    }

    void update(ros::Time t)
    {
        if (t.toSec() - m_last_update_t.toSec() > blink_period)
        {
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