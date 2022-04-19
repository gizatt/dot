#pragma once

#include "ros.h"

class CurrentSensor
{
public:
    const double MAX_CURRENT = 50;                  // Amps
    const double MAX_CURRENT_RAW = 1024;            // AnalogRead output unit
    const double LOW_PASS_RC = 0.05;                // Time constant, seconds
    const double UPDATE_PERIOD = LOW_PASS_RC * 0.2; // Max of 5 sample averaging
    CurrentSensor(ros::NodeHandle &nh, uint8_t pin) : m_pin(pin), m_current(-1.), m_last_update_t(nh.now())
    {
    }

    void update(ros::Time t)
    {
        double dt = t.toSec() - m_last_update_t.toSec();
        if (dt > UPDATE_PERIOD)
        {
            m_last_update_t = t;

            float val = (float)analogRead(m_pin);
            val = MAX_CURRENT * (val / MAX_CURRENT_RAW); // Amps

            if (m_current < 0)
            {
                m_current = val;
            }
            else
            {
                float alpha = dt / (LOW_PASS_RC + dt);
                m_current = (1. - alpha) * m_current + alpha * val;
            }
        }
    }

    inline float get_current()
    {
        return m_current;
    }

private:
    uint8_t m_pin;
    float m_current;
    ros::Time m_last_update_t;
};
