#pragma once

#include "ros.h"
#include "ros/time.h"
#include <dot_msgs/ServoSetTrajectory.h>
#include <sensor_msgs/JointState.h>
#include <Adafruit_PWMServoDriver.h>

#define JOINT_TRAJECTORY_CMD_CHANNEL "due_joint_trajectory_cmd"
const int MICROSECOND_MIN = 200;
const int MICROSECOND_MAX = 3000;
const double SERVO_UPDATE_PERIOD = 0.010; // Every 10 ms, send servo updates, if there are any.
const int SERVO_OUTPUT_DISABLE_PIN = 19;
const int NUM_POSITIONS = 16;
const int MAX_TRAJECTORY_LENGTH = 30;

class PiecewiseTrajectory {
    /**
     * Utility for tracking a piecewise linear trajectory from a JointTrajectory
     * message.
     **/
    private:
        ros::NodeHandle& nh_;
        typedef float data_type_;
        data_type_ data_[NUM_POSITIONS*MAX_TRAJECTORY_LENGTH];
        double breaks_[NUM_POSITIONS];
        unsigned int num_breaks_;
        char err_buffer_[256];
    public:
        PiecewiseTrajectory(ros::NodeHandle& nh);
        bool SetFromMsg(const dot_msgs::ServoSetTrajectory& msg);
        unsigned int get_num_breaks() { return num_breaks_; }
        void get_command_at_time(double t, data_type_ * output_buffer);
};

class ServoSetController {
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

private:
    sensor_msgs::JointState last_command_{};
    float command_buffer_[NUM_POSITIONS];
    PiecewiseTrajectory commanded_trajectory_;
    ros::NodeHandle& nh_;
    ros::Subscriber<dot_msgs::ServoSetTrajectory, ServoSetController> sub_;

    double avg_update_period_ = 1.;

    /* PWM management hardware. */
    Adafruit_PWMServoDriver pwm_;

public:
    ServoSetController(ros::NodeHandle& nh);
    /* Apply a control update for time t. */
    void update(ros::Time t);
    /* Handle a new JointTrajectory message. */
    void trajectory_msg_callback(const dot_msgs::ServoSetTrajectory& msg);
    ros::Time get_last_command_time() {
        return ros::Time(last_command_.header.stamp.sec, last_command_.header.stamp.nsec);
    }
    double get_average_update_rate() {
        return 1. / avg_update_period_;
    }
    const sensor_msgs::JointState& get_last_command(){
        return last_command_;
    }
};