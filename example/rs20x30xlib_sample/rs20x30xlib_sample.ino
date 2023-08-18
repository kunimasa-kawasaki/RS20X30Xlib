/* -------------------------------------

Arduino library for Futaba Serial Servo RS20X, RS30X
rs20x30xlib_sample.ino

Copyright (c) 2023 Kunimasa Kawasaki

------------------------------------- */
#include "rs20x30xlib.h"

int check_servo_id = 1; // use servo id
RS20X30XDeviceIo *rs204;

void setup(){
    Serial1.begin(115200);  // servo Serial: 115200 is init bps
    Serial.begin(115200);   // USB Serial, debug by serial console

    rs204 = new RS20X30XDeviceIo(Serial1);
    delay(1000);
}

void loop(){
    // torque on
    rs204->setTorqueOn(check_servo_id);
    Serial.println((String)"torque on");
    delay(1);

    // get system info
    String model_num = rs204->getModelNumber(check_servo_id);
    String firmware_ver = rs204->getFirmwareVersion(10);
    Serial.println((String)"model   : "+model_num);
    Serial.println((String)"Version : "+firmware_ver);

    int ack = rs204->checkACK(check_servo_id);
    Serial.println((String)"ACK     : "+ack);
    delay(1);

    // set and move goal position
    int move_angle = 90; // [deg]
    int move_time = 1000; // [ms]
    rs204->setGoalPositionInTime(check_servo_id, move_angle, move_time);
    Serial.println((String)"move: "+move_angle+"[deg], "+move_time+"[ms]");
    delay(move_time);

    // get servo status
    RS20X30X_STATUS status = rs204->getStatus(check_servo_id);
    Serial.println((String)"position   : "+status.present_position);
    Serial.println((String)"time       : "+status.present_time);
    Serial.println((String)"speed      : "+status.present_speed);
    Serial.println((String)"current    : "+status.present_current);
    Serial.println((String)"temperature: "+status.present_temperature);
    Serial.println((String)"volts      : "+status.present_volts);

    // set and move goal position
    move_angle = 0;
    move_time = 2000;
    Serial.println((String)"move: "+move_angle+"[deg], "+move_time+"[ms]");
    rs204->setGoalPositionInTime(check_servo_id, move_angle, move_time);
    delay(move_time);

    // torque off
    rs204->setTorqueOff(check_servo_id);
    Serial.println((String)"torque off");
    delay(1000);
}
