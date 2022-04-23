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
#include "IMUST.hpp"
#include "ServoSetController.hpp"

ros::NodeHandle nh;

#ifndef LED_BUILTIN
#define LED_BUILTIN 13
#endif

#define A2D_CURRENT_PIN 24

DebugStrPublisher *debug_publisher;
CurrentSensor *current_sensor;
LEDBlinker *blinker;
StatusSender *status_sender;
IMUST *imu_st;
ServoSetController *servo_set_controller;

void setup()
{
    // initialize LED digital pin as an output.
    //pinMode(LED_BUILTIN, OUTPUT);

    nh.getHardware()->setBaud(9800);
    nh.initNode();

    while (!nh.connected())
    {
        nh.spinOnce();
    }

    debug_publisher = new DebugStrPublisher(nh);
    current_sensor = new CurrentSensor(nh, A2D_CURRENT_PIN);
    imu_st = new IMUST(nh, debug_publisher);
    servo_set_controller = new ServoSetController(nh, debug_publisher);
    status_sender = new StatusSender(nh, current_sensor, imu_st, servo_set_controller, debug_publisher);
    debug_publisher->log("Initialized.");
}

void loop()
{
    auto t = nh.now();

    { // Update all modules.
        current_sensor->update(t);
        //blinker->update(t);
        imu_st->update(t);
        status_sender->update(t);
        servo_set_controller->update(t);
        nh.spinOnce();
    }
}