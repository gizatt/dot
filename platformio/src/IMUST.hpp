#pragma once

#include "ros.h"
#include <Adafruit_LSM6DSOX.h>

#include "DebugStrPublisher.hpp"

class IMUST
{
public:
    const uint8_t PIN_CS = 10;
    const uint8_t PIN_MOSI = 11;
    const uint8_t PIN_MISO = 12;
    const uint8_t PIN_SCK = 13;
    const double UPDATE_PERIOD = 0.01;
    const double RECONNECT_PERIOD = 1.0;

    IMUST(ros::NodeHandle &nh, DebugStrPublisher *debug_publisher = nullptr) : m_debug_publisher(debug_publisher), m_have_imu(false), m_last_update_t(nh.now())
    {
        pinMode(PIN_CS, OUTPUT);
        SPI.begin();
        try_connect();
    }

    bool connected()
    {
        return m_have_imu;
    }

    bool try_connect()
    {
        m_have_imu = m_sox.begin_SPI(PIN_CS, &SPI);
        if (!m_have_imu && m_debug_publisher)
        {
            m_debug_publisher->log("Could not connect to IMU.");
        }
        return m_have_imu;
    }

    void update(ros::Time t)
    {
        double dt = t.toSec() - m_last_update_t.toSec();

        if (m_have_imu && dt >= UPDATE_PERIOD)
        {
            m_sox.getEvent(&m_accel, &m_gyro, &m_temp);
            m_acc_gyro_temp[0] = m_accel.acceleration.x;
            m_acc_gyro_temp[1] = m_accel.acceleration.y;
            m_acc_gyro_temp[2] = m_accel.acceleration.z;
            m_acc_gyro_temp[3] = m_gyro.gyro.x;
            m_acc_gyro_temp[4] = m_gyro.gyro.y;
            m_acc_gyro_temp[5] = m_gyro.gyro.z;
            m_acc_gyro_temp[6] = m_temp.temperature;
            m_last_update_t = t;
        }
        else if (!m_have_imu && dt >= RECONNECT_PERIOD)
        {
            try_connect();
            m_last_update_t = t;
        }
    }

    const float *acceleration()
    {
        return m_acc_gyro_temp + 0;
    }
    const float *rotational_velocity()
    {
        return m_acc_gyro_temp + 3;
    }
    float temperature()
    {
        return m_acc_gyro_temp[6];
    }

private:
    DebugStrPublisher *m_debug_publisher;
    bool m_have_imu;
    ros::Time m_last_update_t;
    Adafruit_LSM6DSOX m_sox;
    // Buffer in Acc xyz, Gyro xyz, Temp order.
    float m_acc_gyro_temp[7];
    // Buffers needed to get data out of the IMU library.
    sensors_event_t m_accel;
    sensors_event_t m_gyro;
    sensors_event_t m_temp;
};
