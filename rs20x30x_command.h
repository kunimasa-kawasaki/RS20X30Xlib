/* -------------------------------------

Arduino library for Futaba Serial Servo RS20X, RS30X
rs20x30xcommand.h

Copyright (c) 2023 Kunimasa Kawasaki

------------------------------------- */
#ifndef RS20X30X_COMMAND_H
#define RS20X30X_COMMAND_H

// Band Rate
#define UART_BAUDRATE_9600	0x00
#define UART_BAUDRATE_14400	0x01
#define UART_BAUDRATE_19200	0x02
#define UART_BAUDRATE_28800	0x03
#define UART_BAUDRATE_38400	0x04
#define UART_BAUDRATE_57600	0x05
#define UART_BAUDRATE_76800	0x06
#define UART_BAUDRATE_115200	0x07
#define UART_BAUDRATE_153600	0x08
#define UART_BAUDRATE_230400	0x09

// --- Flags
#define FLAG_WRITE_ROM		0x40
#define FLAG_SERVO_REBOOT	0x20
#define FLAG_INITIALIZE		0x10
#define FLAG_RETURN_NON		0x00
#define FLAG_RETURN_ACK		0x01
#define FLAG_RETURN_NO00_29	0x03
#define FLAG_RETURN_NO30_59	0x05
#define FLAG_RETURN_NO20_29	0x07
#define FLAG_RETURN_NO42_59	0x09
#define FLAG_RETURN_NO30_41	0x0b
#define FLAG_RETURN_NO60_127	0x0d
#define FLAG_RETURN_SPECIFY	0x0f

// address
#define SYSTEM_MODEL_NUMBER_L		0x00 //*Unchangeable
#define SYSTEM_MODEL_NUMBER_H		0x01 //*Unchangeable
#define SYSTEM_FIRMWARE_VERSION		0x02 //*Unchangeable

// --- ROM
#define	ROM_SERVO_ID			0x04 //*Command mode
#define	ROM_REVERSE			0x05
#define	ROM_BAUDRATE			0x06 //*Command mode
#define	ROM_RETURN_DELAY		0x07 //*Command mode
#define	ROM_CW_ANGLELIMIT_L		0x08
#define	ROM_CW_ANGLELIMIT_H		0x09
#define	ROM_CCW_ANGLELIMIT_L		0x0a
#define	ROM_CCW_ANGLELIMIT_H		0x0b

#define	ROM_TEMPRATURELIMIT_L		0x0e //*Read only
#define	ROM_TEMPRATURELIMIT_H		0x0f //*Read only

#define ROM_TORQUE_IN_SILENCE		0x16 //*PWM mode
#define ROM_WARM_UP_TIME		0x17 //*PWM mode
#define ROM_CW_COMPLIANCE_MARGIN	0x18
#define ROM_CCW_COMPLIANCE_MARGIN	0x19
#define ROM_CW_COMPLIANCE_SLOPE		0x1a
#define ROM_CCW_COMPLIANCE_SLOPE	0x1b
#define ROM_PUNCH_L			0x1c
#define ROM_PUNCH_H			0x1d

// --- RAM
#define RAM_GOAL_POSITION_L		0x1e
#define RAM_GOAL_POSITION_H		0x1f
#define RAM_GOAL_TIME_L			0x20
#define RAM_GOAL_TIME_H			0x21

#define RAM_MAX_TORQUE			0x23
#define RAM_TORQUE_ENABLE		0x24
#define RAM_PID_COEFFICIENT		0x26

#define RAM_PRESENT_POSITION_L		0x2a //*Read only
#define RAM_PRESENT_POSITION_H		0x2b //*Read only
#define RAM_PRESENT_TIME_L		0x2c //*Read only
#define RAM_PRESENT_TIME_H		0x2d //*Read only
#define RAM_PRESENT_SPEED_L		0x2e //*Read only
#define RAM_PRESENT_SPEED_H		0x2f //*Read only
#define RAM_PRESENT_CURRENT_L		0x30 //*Read only
#define RAM_PRESENT_CURRENT_H		0x31 //*Read only
#define RAM_PRESENT_TEMPERATURE_L	0x32 //*Read only
#define RAM_PRESENT_TEMPERATURE_H	0x33 //*Read only
#define	RAM_PRESENT_VOLTS_L		0x34 //*Read only
#define RAM_PRESENT_VOLTS_H		0x35 //*Read only

// --- ID
#define ID_ALL	0xff

// --- Reverse
#define REVERSE_CW	0x00
#define REVERSE_CCW 	0x01

// --- BAUDRATE
#define BAUDRATE_9600	0x00
#define BAUDRATE_14400	0x01
#define BAUDRATE_19200	0x02
#define BAUDRATE_28800	0x03
#define BAUDRATE_38400	0x04
#define BAUDRATE_57600	0x05
#define BAUDRATE_76800	0x06
#define BAUDRATE_115200	0x07
#define BAUDRATE_153600	0x08
#define BAUDRATE_230400	0x09

// --- Torque in silence
#define TORQUE_IN_SILENCE_FREE	0x00
#define TORQUE_IN_SILENCE_KEEP	0x01
#define TORQUE_IN_SILENCE_BRAKE	0x02

// -- TORQUE
#define TORQUE_OFF	0x00
#define TORQUE_ON	0x01
#define TORQUE_BRAKE	0x02

// -- PACKET
#define PACKET_HEADER_1		0xFA
#define PACKET_HEADER_2		0xAF
#define RETURN_PACKET_HEADER_1 	0xFD
#define RETURN_PACKET_HEADER_2 	0xDF

typedef struct rs20x30x_status
{
	float		present_position;
	unsigned int	present_time;
	unsigned int	present_speed;
	unsigned int	present_current;
	unsigned int	present_temperature;
	float		present_volts;
}RS20X30X_STATUS;

// -- LIMIT
#define MOVE_LIMIT_MIN -150 // [deg]
#define MOVE_LIMIT_MAX 150  // [deg]

#define MOVE_LIMIT_SPEED_MAX 0.24 // [deg per ms]

#define MOVE_LIMIT_WARMUPTIME_MAX 	2550 // [ms]
#define MOVE_LIMIT_COMPLIANCE_MARGIN 	25.5 // [deg]

#endif
