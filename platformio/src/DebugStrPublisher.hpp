#pragma once

#include "ros.h"
#include <std_msgs/Header.h>

class DebugStrPublisher
{
    /*
     * Packs timestamped debug strs into a Header msg,
     * putting the debug str into the frame_id string field.
     */
public:
    const static int BUF_LEN = 1000;
    DebugStrPublisher(ros::NodeHandle &nh) : m_nh(nh),
                                             m_pub("speck/debug", &m_msg)
    {
        nh.advertise(m_pub);
    }
    template <typename... Args>
    void log(Args &&... args)
    {
        snprintf(m_buf, BUF_LEN, args...);
        m_msg.stamp = m_nh.now();
        m_msg.frame_id = m_buf;
        m_pub.publish(&m_msg);
    }

private:
    ros::NodeHandle &m_nh;
    std_msgs::Header m_msg;
    ros::Publisher m_pub;
    char m_buf[BUF_LEN];
};
// Singleton debugPublisher
DebugStrPublisher *debugPublisherSingleton;
void create_debug_publisher_singleton(ros::NodeHandle &nh)
{
    debugPublisherSingleton = new DebugStrPublisher(nh);
}