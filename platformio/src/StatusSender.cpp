#include "StatusSender.hpp"

void StatusSender::update(ros::Time t)
{
    double dt = t.toSec() - m_last_update_t.toSec();
    m_last_update_t = t;
    if (m_avg_update_period < 0 || m_avg_update_period > 1.)
    {
        m_avg_update_period = dt;
    }
    else
    {
        float alpha = dt / (LOW_PASS_RC + dt);
        m_avg_update_period = m_avg_update_period * (1. - alpha) + alpha * dt;
    }

    if (t.toSec() - m_last_send_t.toSec() >= SEND_PERIOD)
    {
        m_last_send_t = t;

        // Format + send status message.
        m_status_msg.header.stamp = t;
        float current = m_current_sensor->get_current();
        m_status_msg.battery_current = current;

        m_status_msg.average_loop_rate = 1. / m_avg_update_period;

        // Populate IMU info.
        m_status_msg.has_imu = m_imu_st->connected();
        if (m_status_msg.has_imu)
        {
            for (int i = 0; i < 3; i++)
            {
                m_status_msg.imu_acc[i] = *(m_imu_st->acceleration() + i);
                m_status_msg.imu_gyro[i] = *(m_imu_st->rotational_velocity() + i);
            }
            m_status_msg.imu_temp = m_imu_st->temperature();
        }
        m_status_msg.last_command_time = m_servo_set_controller->get_last_command_time();
        m_status_msg.last_command = m_servo_set_controller->get_last_command();

        m_status_msg.last_error_msg = m_debug_publisher->buf();

        m_pub.publish(&m_status_msg);
    }
}