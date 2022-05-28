#pragma once

#include "ros.h"
#include "ros/time.h"
#include <sensor_msgs/JointState.h>
#include <dot_msgs/ServoSetTrajectory.h>

#include <Adafruit_PWMServoDriver.h>
#include "DebugStrPublisher.hpp"

const int NUM_POSITIONS = 16;

class PiecewiseTrajectory
{
    /**
     * Utility for tracking a piecewise linear trajectory from a JointTrajectory
     * message.
     **/

public:
    using T = float;
    const static unsigned int MAX_TRAJECTORY_LENGTH = 30;

    PiecewiseTrajectory(ros::NodeHandle &nh, DebugStrPublisher *debug_publisher = nullptr) : m_nh(nh), m_debug_publisher(debug_publisher), m_num_breaks(0) {}
    /*
     * Populate this trajectory from a message. If q0 is not null, then it should be a NUM_POSITIONS
     * array of scalars that are used as the pose at knot 0 if a break at time 0 is *not*
     * provided in the message. (This allows natural slewing from current commanded posture.)
     */
    bool SetFromMsg(const dot_msgs::ServoSetTrajectory &msg, PiecewiseTrajectory::T *q0 = nullptr);
    unsigned int get_num_breaks() { return m_num_breaks; }
    void get_command_at_time(double t, T *output_buffer);

private:
    ros::NodeHandle &m_nh;
    DebugStrPublisher *m_debug_publisher;
    T m_data[NUM_POSITIONS * MAX_TRAJECTORY_LENGTH];
    double m_breaks[MAX_TRAJECTORY_LENGTH];
    unsigned int m_num_breaks;
};

class ServoSetController
{
    /**
     * 
     *  Controls the servo set on Dot.
     * 
     *  Subscribes to channel JOINT_TRAJECTORY_CMD_CHANNEL to receive
     *  ServoSetTrajectory messages, which specify a series of time points
     *  (with time relative to roscore time), each of which should have
     *  a number of positions matching the number of servos in this controller,
     *  expressing desired servo microsecond commands.
     *  This controller will spool those commands out to the PWM board (using
     *  linear interpolation between command points for smoothing), subject to
     *  some simple prior microsecond bounds.
     */

public:
    const char *JOINT_TRAJECTORY_CMD_CHANNEL = "speck/joint_trajectory_cmd";
    const unsigned int MICROSECOND_MIN = 200;
    const unsigned int MICROSECOND_MAX = 3000;
    const double SERVO_UPDATE_PERIOD = 0.010; // Every 10 ms, send servo updates, if there are any.
    const uint8_t SERVO_OUTPUT_DISABLE_PIN = 23;

    ServoSetController(ros::NodeHandle &nh, DebugStrPublisher *debug_publisher = nullptr);
    /* Apply a control update for time t. */
    void update(ros::Time t);
    /* Handle a new JointTrajectory message. */
    void trajectory_msg_callback(const dot_msgs::ServoSetTrajectory &msg);
    ros::Time get_last_command_time()
    {
        return ros::Time(m_last_command.header.stamp.sec, m_last_command.header.stamp.nsec);
    }
    double get_average_update_rate()
    {
        return 1. / m_avg_update_period;
    }
    const sensor_msgs::JointState &get_last_command()
    {
        return m_last_command;
    }

private:
    ros::NodeHandle &m_nh;
    ros::Subscriber<dot_msgs::ServoSetTrajectory, ServoSetController> m_sub;
    DebugStrPublisher *m_debug_publisher;

    /* PWM management hardware. */
    Adafruit_PWMServoDriver m_pwm;

    sensor_msgs::JointState m_last_command{};
    float m_command_buffer[NUM_POSITIONS];
    PiecewiseTrajectory m_commanded_trajectory;

    double m_avg_update_period = 1.;
};