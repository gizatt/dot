/**
 * Blink
 *
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */
#include "Arduino.h"

#include "ros.h"
#include "dot_msgs/SpeckStatus.h"
#include <stdio.h>
#include <stdarg.h>

#include "DebugStrPublisher.hpp"
#include "LEDBlinker.hpp"
#include "CurrentSensor.hpp"
#include "StatusSender.hpp"

ros::NodeHandle nh;

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#define A2D_CURRENT_PIN 24

CurrentSensor *current_sensor;
LEDBlinker *blinker;
StatusSender *status_sender;

void setup()
{
    // initialize LED digital pin as an output.
    pinMode(LED_BUILTIN, OUTPUT);

    nh.getHardware()->setBaud(9800);
    nh.initNode();

    create_debug_publisher_singleton(nh);
    blinker = new LEDBlinker(nh, LED_BUILTIN);
    current_sensor = new CurrentSensor(nh, A2D_CURRENT_PIN);
    status_sender = new StatusSender(nh, current_sensor);
    debugPublisherSingleton->log("Initialized.");
}

void loop()
{
    auto t = nh.now();

    { // Update all modules.
        current_sensor->update(t);
        blinker->update(t);
        status_sender->update(t);
        nh.spinOnce();
    }
}