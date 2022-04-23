#pragma once

#include "ros.h"
#include "dot_msgs/SpeckStatus.h"
#include "CurrentSensor.hpp"
#include "IMUST.hpp"
#include "ServoSetController.hpp"
#include "DebugStrPublisher.hpp"

class StatusSender
{
public:
    const double SEND_PERIOD = 0.05;
    const double LOW_PASS_RC = 0.5; // Time constant, seconds

    StatusSender(ros::NodeHandle &nh, CurrentSensor *current_sensor,
                 IMUST *imu_st, ServoSetController *servo_set_controller,
                 DebugStrPublisher *debug_publisher) : m_pub("speck/status", &m_status_msg),
                                                       m_current_sensor(current_sensor),
                                                       m_imu_st(imu_st),
                                                       m_servo_set_controller(servo_set_controller),
                                                       m_debug_publisher(debug_publisher),
                                                       m_avg_update_period(-1.),
                                                       m_last_update_t(nh.now()),
                                                       m_last_send_t(nh.now())
    {
        nh.advertise(m_pub);
    }

    void update(ros::Time t);

private:
    ros::Publisher m_pub;
    dot_msgs::SpeckStatus m_status_msg;

    CurrentSensor *m_current_sensor;
    IMUST *m_imu_st;
    ServoSetController *m_servo_set_controller;
    DebugStrPublisher *m_debug_publisher;

    float m_avg_update_period;
    ros::Time m_last_update_t;
    ros::Time m_last_send_t;
};
