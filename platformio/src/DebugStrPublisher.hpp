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
    DebugStrPublisher(ros::NodeHandle &nh) : m_nh(nh)
    {
    }
    template <typename... Args>
    void log(Args &&... args)
    {
        snprintf(m_buf, BUF_LEN, args...);
        m_nh.loginfo(m_buf);
    }

    template <typename... Args>
    void logdebug(Args &&... args)
    {
        snprintf(m_buf, BUF_LEN, args...);
        m_nh.logdebug(m_buf);
    }

    template <typename... Args>
    void loginfo(Args &&... args)
    {
        snprintf(m_buf, BUF_LEN, args...);
        m_nh.loginfo(m_buf);
    }

    template <typename... Args>
    void logwarn(Args &&... args)
    {
        snprintf(m_buf, BUF_LEN, args...);
        m_nh.logwarn(m_buf);
    }

    template <typename... Args>
    void logerror(Args &&... args)
    {
        snprintf(m_buf, BUF_LEN, args...);
        m_nh.logerror(m_buf);
    }

    const char *buf()
    {
        return m_buf;
    }

private:
    ros::NodeHandle &m_nh;
    char m_buf[BUF_LEN];
};
