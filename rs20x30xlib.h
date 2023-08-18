/* -------------------------------------

Arduino library for Futaba Serial Servo RS20X, RS30X
rs20x30xlib.h

Copyright (c) 2023 Kunimasa Kawasaki

------------------------------------- */
#ifndef RS20X30XLIB_H
#define RS20X30XLIB_H

#include "Arduino.h"
#include "rs20x30x_command.h"

class RS20X30XDeviceIo
{
private:
    HardwareSerial *serial;

    int commandSend( unsigned char *sbuf, unsigned int ssize, unsigned char *rbuff, unsigned int rsize ); //return
    int commandSend( unsigned char *sbuf, unsigned int ssize ); // no return

    int checksum( unsigned char *send, unsigned char len );
    int writePram( unsigned char id, unsigned char flag, unsigned char address, unsigned char len, unsigned char count, unsigned char * data );
    int readPram( unsigned char id, unsigned char flag, unsigned char address, unsigned char len, unsigned char *rdata, unsigned int rsize );
    int Buff2Status( unsigned char flag, unsigned char *rbuff );

public:
    RS20X30XDeviceIo(HardwareSerial &_serial);
    ~RS20X30XDeviceIo();
    /* --- system function --- */
    int flushROM( unsigned char id );
    int reboot( unsigned char id );
    int initializeROM( unsigned char id );
    bool checkACK( unsigned char id );

    /* --- get function --- */
    // --- system
    String getModelNumber( unsigned char id );
    String getFirmwareVersion( unsigned char id );

    // ROM
    // --- info
    int getServoID( unsigned char id );
    int getReverse( unsigned char id );
    int getBaudRate( unsigned char id );
    int getReturnDelay( unsigned char id );
    // --- limit
    int getCWAngleLimit( unsigned char id);
    int getCCWAngleLimit( unsigned char id );
    int getTemperatureLimit( unsigned char id );
    // --- pwm
    int getTorqueInSilence( unsigned char id );
    int getWarmUpTime( unsigned char id );
    // --- compliance
    int getCWComplianceMargin( unsigned char id );
    int getCCWComplianceMargin( unsigned char id );
    int getCWComplianceSlope( unsigned char id );
    int getCCWComplianceSlope( unsigned char id );
    int getPunch( unsigned char id );

    // RAM;
    // --- ctl
    int getGoalPosition( unsigned char id );
    int getGoalTime( unsigned char id );
    int getMaxTorque( unsigned char id );
    int getTorqueEnable( unsigned char id );
    int getPIDCoefficient( unsigned char id );
    // --- status
    RS20X30X_STATUS getStatus( unsigned char id );
        float getPosition( unsigned char id );  //位置
        int getTime( unsigned char id );	//経過時間
        int getSpeed( unsigned char id );	//速度
        int getCurrent( unsigned char id );	//電流
        int getTemperature( unsigned char id ); //温度
        float getVoltage( unsigned char id );   //電圧

    /* --- set function --- */
    // ROM
    // --- info
    int setServoID( unsigned char id, unsigned char new_id );
    int setReverse( unsigned char id, unsigned char direction );
        int setReverseCW( unsigned char id );
        int setReverseCCW( unsigned char id );
    int setBaudRate( unsigned char id, unsigned char bps );
        int setBaudRate9600( unsigned char id );
        int setBaudRate14400( unsigned char id );
        int setBaudRate19200( unsigned char id );
        int setBaudRate28800( unsigned char id );
        int setBaudRate38400( unsigned char id );
        int setBaudRate57600( unsigned char id );
        int setBaudRate76800( unsigned char id );
        int setBaudRate115200( unsigned char id );
        int setBaudRate153600( unsigned char id );
        int setBaudRate230400( unsigned char id );
    int setReturnDelay( unsigned char id, unsigned int time );   // unit of time 50[us]
    int setAngleLimit( unsigned char id, short angle );   // unit of angle 0.1[deg]

    // --- pwm
    int setTorqueInSilence( unsigned char id, unsigned char mode );
        int setTorqueInSilenceFree( unsigned char id );
        int setTorqueInSilenceKeep( unsigned char id );
        int setTorqueInSilenceBrake( unsigned char id );
    int setWarmUpTime( unsigned char id, unsigned int time );    // unit of time 10[ms]

    // --- compliance
    int setCompliance( unsigned char id, short cw_m_angle, short ccw_m_angle, unsigned char cw_s_angle, unsigned char ccw_s_angle, unsigned char p_percent );
    int setCWComplianceMargin( unsigned char id, short angle );   // unit of angle 0.1[deg]
    int setCCWComplianceMargin( unsigned char id, short angle );  // unit of angle 0.1[deg]
    int setCWComplianceSlope( unsigned char id, unsigned char angle );    // unit of angle 1[deg]
    int setCCWComplianceSlope( unsigned char id, unsigned char angle );   // unit of angle 1[deg]
    int setPunch( unsigned char id, unsigned char percent );              // unit of percent 1/255

    // --- goal position
    int setGoalPosition( unsigned char id, short angle);                     		// unit of angle [deg]
    int setGoalPositionInTime( unsigned char id, short angle, unsigned int time);     	// unit of angle [deg], time 10[ms]

    // --- torque
    int setMaxTorque( unsigned char id, unsigned char percent ); // unit of percent 1/100
    int setTorqueMode(unsigned char id, unsigned char mode );
        int setTorqueOff( unsigned char id );
        int setTorqueOn( unsigned char id );
        int setTorqueBrake( unsigned char id );

    // --- PID
    int setPIDCoefficient( unsigned char id, unsigned int percent ); // unit of percent 1/100

    // long packet
    //int setGoalPositionInTime(short id[], short angle[], short time,  int n );
};

#endif
