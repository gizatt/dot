
#include "servo_set_controller.hpp"

PiecewiseTrajectory::PiecewiseTrajectory(ros::NodeHandle& nh) :
    nh_(nh), num_breaks_(0)
{
}

bool PiecewiseTrajectory::SetFromMsg(const dot_msgs::ServoSetTrajectory& msg){
    // Reject trajectory if it doesn't pass sanity checks;
    // otherwise copy it over.
    if (msg.num_breaks >= MAX_TRAJECTORY_LENGTH){
        sprintf(err_buffer_, "Rejecting trajectory with too many points: %d vs limit %d.", msg.num_breaks, MAX_TRAJECTORY_LENGTH);
        nh_.logerror(err_buffer_);
        return false;
    }
    // Check that point set is the right size.
    if (msg.num_positions != NUM_POSITIONS){
        sprintf(err_buffer_, "Rejecting trajectory with bad point size: %d vs expected %d.",
                msg.num_positions,
                NUM_POSITIONS);
        nh_.logerror(err_buffer_); 
        return false;
    }
    // Check that arrays are reasonable sizes.
    if (msg.breaks_from_start_length != msg.num_breaks){
        nh_.logerror("Rejecting trajectory with bad breaks length."); 
        return false;
    }
    if (msg.data_length != msg.num_breaks*msg.num_positions){
        nh_.logerror("Rejecting trajectory with bad data length."); 
        return false;
    }
    // Check that times are monotonically increasing.
    for (int i = 0; i < msg.num_breaks - 1; i++){
        if (msg.breaks_from_start[i] >= msg.breaks_from_start[i+1]){
            nh_.logerror("Rejecting trajectory with non-monotonic times.");
            return false;
        }
    }
    // Copy data over into our internal buffer for the piecewise trajectory.
    double start_time = msg.header.stamp.toSec();
    for (unsigned int i = 0; i < msg.num_breaks; i++){
        breaks_[i] = start_time + msg.breaks_from_start[i];
        for (unsigned int j = 0; j < msg.num_positions; j++){
            // Convert from uint16 to float.
            data_[i*NUM_POSITIONS + j] = msg.data[i*NUM_POSITIONS + j];
        }
    }
    num_breaks_ = msg.num_breaks;
    return true;
}

void PiecewiseTrajectory::get_command_at_time(double t, data_type_ * output_buffer){
    // If we have zero breaks, set output buffer to all -1.
    if (num_breaks_ == 0){
        for (int i = 0; i < NUM_POSITIONS; i++){
            output_buffer[i] = -1;
        }
        return;
    }
    if (num_breaks_ == 1){
        for (int i = 0; i < NUM_POSITIONS; i++){
            output_buffer[i] = data_[i];
        }
        return;
    }
    // If this time is before or after the first or last break,
    // copy the corresponding first or last point into the output buffer.
    if (t <= breaks_[0]){
        for (int i = 0; i < NUM_POSITIONS; i++){
            output_buffer[i] = data_[i];
        }
        return;
    }
    if (t >= breaks_[num_breaks_ - 1]) {
        for (int i = 0; i < NUM_POSITIONS; i++){
            output_buffer[i] = data_[i + NUM_POSITIONS * (num_breaks_ - 1)];
        }
        return;
    }

    // Find the boundary times and interpolate between them.
    for (int k = 0; k < num_breaks_ - 1; k++){
        if (breaks_[k] <= t && breaks_[k + 1] >= t){
            double w1 = t - breaks_[k];
            double w2 = breaks_[k + 1] - t;
            double dt = w1 + w2;
            w1 /= dt;
            w2 /= dt;
            // Now w1 + w2 = 1, and they can be used to blend the
            // two points.
            for (int i = 0; i < NUM_POSITIONS; i++){
                output_buffer[i] = data_[i + NUM_POSITIONS * k]*w2 + data_[i + NUM_POSITIONS * (k + 1)]*w1;
            }
            return;
        }
    }
    nh_.logerror("This should be unreachable.");
}


void ServoSetController::trajectory_msg_callback(
    const dot_msgs::ServoSetTrajectory& msg)
{
    char buf[256];
    sprintf(buf, "Got message with delay %f.", nh_.now().toSec() - msg.header.stamp.toSec());
    nh_.loginfo(buf);
    commanded_trajectory_.SetFromMsg(msg);
}

ServoSetController::ServoSetController(ros::NodeHandle& nh) :
    nh_(nh),
    sub_(JOINT_TRAJECTORY_CMD_CHANNEL, &ServoSetController::trajectory_msg_callback, this),
    pwm_(Adafruit_PWMServoDriver()),
    commanded_trajectory_(nh)
{
    nh.subscribe(sub_);

    /* Enable the PWM hardware. */  
    pwm_.begin();
    pwm_.setOscillatorFrequency(27000000);
    pwm_.setPWMFreq(50);  // Analog servos run at ~50 Hz updates
    pinMode(SERVO_OUTPUT_DISABLE_PIN, OUTPUT);
    digitalWrite(SERVO_OUTPUT_DISABLE_PIN, 0);

    /* Initialize last command buffer. */
    ros::Time now = nh_.now();
    last_command_.header.stamp.sec = now.sec;
    last_command_.header.stamp.nsec = now.nsec;
    last_command_.position_length = NUM_POSITIONS;
    // Hook up command message to our preallocated float command buffer;
    last_command_.position = command_buffer_;
    for (int k = 0; k < NUM_POSITIONS; k++)
        last_command_.position[k] = 0.;
}

void ServoSetController::update(ros::Time t){
    // Only bother doing computation if we're due for
    // a servo update.
    double t_double = t.toSec();
    double last_servo_update_time = get_last_command_time().toSec();
    if (t_double < last_servo_update_time || t_double > last_servo_update_time + SERVO_UPDATE_PERIOD){
        // Get the current command.
        if (commanded_trajectory_.get_num_breaks() == 0){
            // If we have an empty trajectory, "turn off" the servos
            // (which is a little dubious) by indicting commands -1.
            for (int i = 0; i < NUM_POSITIONS; i++)
                command_buffer_[i] = -1;
        } else {
            commanded_trajectory_.get_command_at_time(t_double, &command_buffer_[0]);
        }
        // Enact the command, updating last-applied-command in-place.
        for (int i = 0; i < NUM_POSITIONS; i++){
            if (command_buffer_[i] >= 0){
                if (command_buffer_[i] >= MICROSECOND_MIN &&
                    command_buffer_[i] <= MICROSECOND_MAX){
                    pwm_.writeMicroseconds(i, command_buffer_[i]); // commit the legal command
                } else {
                    command_buffer_[i] = -2; // error state
                }
            }
            if (command_buffer_[i] < 0){
                pwm_.setPWM(i, 0, 0); // Disable servo. Does this work?
            }
        }

        // Update control rate estimate.
        avg_update_period_ = avg_update_period_ * 0.95 + 0.05 * (t_double - last_servo_update_time);

        // Update last-applied-command time to this time.
        last_command_.header.stamp.sec = t.sec;
        last_command_.header.stamp.nsec = t.nsec;

    }

}
