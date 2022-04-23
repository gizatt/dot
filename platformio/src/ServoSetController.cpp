
#include "ServoSetController.hpp"
#include "DebugStrPublisher.hpp"

bool PiecewiseTrajectory::SetFromMsg(const dot_msgs::ServoSetTrajectory &msg)
{
    // Reject trajectory if it doesn't pass sanity checks;
    // otherwise copy it over.
    if (msg.num_breaks >= MAX_TRAJECTORY_LENGTH)
    {
        if (m_debug_publisher)
            m_debug_publisher->logerror("Rejecting trajectory with too many points: %u vs limit %u.", msg.num_breaks, MAX_TRAJECTORY_LENGTH);
        return false;
    }
    // Check that point set is the right size.
    if (msg.num_positions != NUM_POSITIONS)
    {
        if (m_debug_publisher)
            m_debug_publisher->logerror("Rejecting trajectory with bad point size: %u vs expected %u.",
                                        msg.num_positions,
                                        NUM_POSITIONS);
        return false;
    }
    // Check that arrays are reasonable sizes.
    if (msg.breaks_from_start_length != msg.num_breaks)
    {
        if (m_debug_publisher)
            m_debug_publisher->logerror("Rejecting trajectory with bad breaks length.");
        return false;
    }
    if (msg.data_length != msg.num_breaks * msg.num_positions)
    {
        if (m_debug_publisher)
            m_debug_publisher->logerror("Rejecting trajectory with bad data length.");
        return false;
    }
    // Check that times are monotonically increasing.
    for (unsigned int i = 1; i < msg.num_breaks; i++)
    {
        if (msg.breaks_from_start[i - 1] >= msg.breaks_from_start[i])
        {
            if (m_debug_publisher)
                m_debug_publisher->logerror("Rejecting trajectory with non-monotonic times.");
            return false;
        }
    }
    // Copy data over into our internal buffer for the piecewise trajectory.
    double start_time = msg.header.stamp.toSec();
    for (unsigned int i = 0; i < msg.num_breaks; i++)
    {
        m_breaks[i] = start_time + msg.breaks_from_start[i];
        for (unsigned int j = 0; j < msg.num_positions; j++)
        {
            // Convert from uint16 to float.
            m_data[i * NUM_POSITIONS + j] = msg.data[i * NUM_POSITIONS + j];
        }
    }
    m_num_breaks = msg.num_breaks;
    return true;
}

void PiecewiseTrajectory::get_command_at_time(double t, PiecewiseTrajectory::T *output_buffer)
{
    // If we have zero breaks, set output buffer to all -1.
    if (m_num_breaks == 0)
    {
        for (int i = 0; i < NUM_POSITIONS; i++)
        {
            output_buffer[i] = -1;
        }
        return;
    }
    if (m_num_breaks == 1)
    {
        for (int i = 0; i < NUM_POSITIONS; i++)
        {
            output_buffer[i] = m_data[i];
        }
        return;
    }
    // If this time is before or after the first or last break,
    // copy the corresponding first or last point into the output buffer.
    if (t <= m_breaks[0])
    {
        for (int i = 0; i < NUM_POSITIONS; i++)
        {
            output_buffer[i] = m_data[i];
        }
        return;
    }
    if (t >= m_breaks[m_num_breaks - 1])
    {
        for (int i = 0; i < NUM_POSITIONS; i++)
        {
            output_buffer[i] = m_data[i + NUM_POSITIONS * (m_num_breaks - 1)];
        }
        return;
    }

    // Find the boundary times and interpolate between them.
    for (unsigned int k = 0; k < m_num_breaks - 1; k++)
    {
        if (m_breaks[k] <= t && m_breaks[k + 1] >= t)
        {
            double w1 = t - m_breaks[k];
            double w2 = m_breaks[k + 1] - t;
            double dt = w1 + w2;
            w1 /= dt;
            w2 /= dt;
            // Now w1 + w2 = 1, and they can be used to blend the
            // two points.
            for (int i = 0; i < NUM_POSITIONS; i++)
            {
                output_buffer[i] = m_data[i + NUM_POSITIONS * k] * w2 + m_data[i + NUM_POSITIONS * (k + 1)] * w1;
            }
            return;
        }
    }
    if (m_debug_publisher)
        m_debug_publisher->logerror("This should be unreachable.");
}

void ServoSetController::trajectory_msg_callback(
    const dot_msgs::ServoSetTrajectory &msg)
{
    if (m_debug_publisher)
        m_debug_publisher->logdebug("Got message with delay %f.", m_nh.now().toSec() - msg.header.stamp.toSec());
    m_commanded_trajectory.SetFromMsg(msg);
}

ServoSetController::ServoSetController(ros::NodeHandle &nh, DebugStrPublisher *debug_publisher) : m_nh(nh),
                                                                                                  m_sub(JOINT_TRAJECTORY_CMD_CHANNEL, &ServoSetController::trajectory_msg_callback, this),
                                                                                                  m_debug_publisher(debug_publisher),
                                                                                                  m_pwm(Adafruit_PWMServoDriver()),
                                                                                                  m_commanded_trajectory(nh, debug_publisher)
{
    m_nh.subscribe(m_sub);

    /* Enable the PWM hardware. */
    m_pwm.begin();
    m_pwm.setOscillatorFrequency(27000000);
    m_pwm.setPWMFreq(50); // Analog servos run at ~50 Hz updates
    pinMode(SERVO_OUTPUT_DISABLE_PIN, OUTPUT);
    digitalWrite(SERVO_OUTPUT_DISABLE_PIN, 0);

    /* Initialize last command buffer. */
    ros::Time now = m_nh.now();
    m_last_command.header.stamp = now;
    m_last_command.position_length = NUM_POSITIONS;
    // Hook up command message to our preallocated float command buffer;
    m_last_command.position = m_command_buffer;
    for (int k = 0; k < NUM_POSITIONS; k++)
        m_last_command.position[k] = 0.;
}

void ServoSetController::update(ros::Time t)
{
    // Only bother doing computation if we're due for
    // a servo update.
    double t_double = t.toSec();
    double last_servo_update_time = get_last_command_time().toSec();
    if (t_double < last_servo_update_time || t_double > last_servo_update_time + SERVO_UPDATE_PERIOD)
    {
        // Get the current command.
        if (m_commanded_trajectory.get_num_breaks() == 0)
        {
            // If we have an empty trajectory, "turn off" the servos
            // (which is a little dubious) by indicting commands -1.
            for (int i = 0; i < NUM_POSITIONS; i++)
                m_command_buffer[i] = -1;
        }
        else
        {
            m_commanded_trajectory.get_command_at_time(t_double, &m_command_buffer[0]);
        }
        // Enact the command, updating last-applied-command in-place.
        for (int i = 0; i < NUM_POSITIONS; i++)
        {
            if (m_command_buffer[i] >= 0)
            {
                if (m_command_buffer[i] >= MICROSECOND_MIN &&
                    m_command_buffer[i] <= MICROSECOND_MAX)
                {
                    m_pwm.writeMicroseconds(i, m_command_buffer[i]); // commit the legal command
                }
                else
                {
                    m_command_buffer[i] = -2; // error state
                }
            }
            if (m_command_buffer[i] < 0)
            {
                m_pwm.setPWM(i, 0, 0); // Disable servo. Does this work?
            }
        }

        // Update control rate estimate.
        m_avg_update_period = m_avg_update_period * 0.95 + 0.05 * (t_double - last_servo_update_time);

        // Update last-applied-command time to this time.
        m_last_command.header.stamp = t;
    }
}
