/* -------------------------------------

Arduino library for Futaba Serial Servo RS20X, RS30X
rs20x30xlib.cpp

Copyright (c) 2023 Kunimasa Kawasaki

------------------------------------- */

#include "rs20x30xlib.h"

RS20X30XDeviceIo::RS20X30XDeviceIo(HardwareSerial &_serial){
    serial = &_serial;
}

RS20X30XDeviceIo::~RS20X30XDeviceIo()
{
}

int RS20X30XDeviceIo::commandSend(unsigned char *sbuff, unsigned int ssize, unsigned char *rbuff, unsigned int rsize) {
    // send: return status command
    int rtn = serial->write(sbuff, ssize);
    serial->flush();

    // read echo packet
    unsigned char tmp_buff[ssize];
    int tmp_size = serial->readBytes(tmp_buff, ssize);

    // read return packet
    int chk_size = serial->readBytes(rbuff, rsize);
    if((chk_size != rsize || rbuff[0] != RETURN_PACKET_HEADER_1 || rbuff[1] != RETURN_PACKET_HEADER_2) && rsize != 1){
        for(int nth = 0; nth < rsize; nth++) rbuff[nth] = 0;
        rtn = -1;
    }

    return rtn;
}

int RS20X30XDeviceIo::commandSend(unsigned char *sbuff, unsigned int ssize) {
    int rtn = serial->write(sbuff, ssize);
    serial->flush();
    // read echo packet
    unsigned char tmp_buff[ssize];
    int tmp_size = serial->readBytes(tmp_buff, ssize);

    return rtn;
}

int RS20X30XDeviceIo::checksum(unsigned char *send, unsigned char len) {
	int sum = 0;
	for(int nth=2; nth < len; nth++){
		sum ^= send[nth];
	}
	return sum;
}

int RS20X30XDeviceIo::writePram(unsigned char id, unsigned char flag, unsigned char address, unsigned char len, unsigned char count, unsigned char *data) {
    int rtn;
    unsigned int ssize = len*count + 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    unsigned short int sum;
    unsigned char send[ssize];
    sum = 0;
    int n = 0;
    send[n] = PACKET_HEADER_1;
    send[++n] = PACKET_HEADER_2;
    send[++n] = id;
	send[++n] = flag;
	send[++n] = address;
	send[++n] = len;
	send[++n] = count;
    for (int i = 0; i < len*count; i++)
        send[++n] = data[i];
    sum = checksum(send,len*count+7);
    send[++n] = sum;

    rtn = commandSend(send, ssize); // no return status

    return rtn;
}

int RS20X30XDeviceIo::readPram(unsigned char id, unsigned char flag, unsigned char address, unsigned char len, unsigned char *rdata, unsigned int rsize) {
    int rtn;
    unsigned short int sum = 0;
    unsigned char send[8];
    int n = 0;
    send[n] = PACKET_HEADER_1;
    send[++n] = PACKET_HEADER_2;
    send[++n] = id;
	send[++n] = flag;
	send[++n] = address;
	send[++n] = len;
	send[++n] = 0x00; // Cnt
    sum = checksum(send, 7);
    send[++n] = sum;

	// on return packet
    rtn = commandSend(send, 8, rdata, rsize);

    return rtn;
}

int RS20X30XDeviceIo::Buff2Status(unsigned char flag, unsigned char *rbuff){
    // set read target num
    int target_num = 7 + (flag - RAM_PRESENT_POSITION_L);

    // read data
    int target_data = 0;
    target_data = rbuff[target_num+1]; // high
    target_data = target_data << 8;
    target_data |= rbuff[target_num]; // low

    return target_data;
}
/* system function */
int RS20X30XDeviceIo::flushROM( unsigned char id ){
    return writePram(id, FLAG_WRITE_ROM, 0xFF, 0, 0, 0);
}
int RS20X30XDeviceIo::reboot( unsigned char id ){
    return writePram(id, FLAG_SERVO_REBOOT, 0xFF, 0, 0, 0);
}
int RS20X30XDeviceIo::initializeROM( unsigned char id ){
    return writePram(id, FLAG_INITIALIZE, 0xFF, 0xFF, 0, 0);
}
bool RS20X30XDeviceIo::checkACK( unsigned char id ){
    int rsize = 1; // *ACK only
    unsigned char rbuff[rsize];
    int rtn = readPram(id, FLAG_RETURN_ACK, 0xFF, 1, rbuff, rsize);

    bool ack = false;
    if(rbuff[0] == 0x07) ack = true;
    return ack;
}


/* get function */
// --- system
String RS20X30XDeviceIo::getModelNumber( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 2; // Model Number L + H
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, SYSTEM_MODEL_NUMBER_L, data_size, rbuff, rsize);
    String model_num = String(rbuff[8], HEX) + String(rbuff[7], HEX);
    return model_num;
}
String RS20X30XDeviceIo::getFirmwareVersion( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 1; // Firmware Version
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, SYSTEM_FIRMWARE_VERSION, data_size, rbuff, rsize);
    String f_ver = String(rbuff[7], HEX);
    return f_ver;
}

// --- --- ROM
// --- info
int RS20X30XDeviceIo::getServoID( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 1; // Servo ID
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, ROM_SERVO_ID, data_size, rbuff, rsize);
    int read_data = rbuff[7];
    return read_data;
}
int RS20X30XDeviceIo::getReverse( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 1; // Reverse
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, ROM_REVERSE, data_size, rbuff, rsize);
    int read_data = rbuff[7];
    return read_data;
}
int RS20X30XDeviceIo::getBaudRate( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 1; // Baud Rate
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, ROM_BAUDRATE, data_size, rbuff, rsize);
    int read_data = rbuff[7];
    return read_data;
}
int RS20X30XDeviceIo::getReturnDelay( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 1; // Return Delay
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, ROM_RETURN_DELAY, data_size, rbuff, rsize);
    int read_data = rbuff[7];
    return read_data;
}

// --- limit
int RS20X30XDeviceIo::getCWAngleLimit( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 2; // CW Angle Limit L + H
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, ROM_CW_ANGLELIMIT_L, data_size, rbuff, rsize);
    int read_data = 0;
    read_data = rbuff[8]; // high
    read_data = read_data << 8;
    read_data |= rbuff[7]; // low
    return read_data;
}
int RS20X30XDeviceIo::getCCWAngleLimit( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 2; // CCW Angle Limit L + H
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, ROM_CCW_ANGLELIMIT_L, data_size, rbuff, rsize);
    int read_data = 0;
    read_data = rbuff[8]; // high
    read_data = read_data << 8;
    read_data |= rbuff[7]; // low
    return read_data;
}
int RS20X30XDeviceIo::getTemperatureLimit( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 2; // Temperature Limit L + H
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, ROM_TEMPRATURELIMIT_L, data_size, rbuff, rsize);
    int read_data = 0;
    read_data = rbuff[8]; // high
    read_data = read_data << 8;
    read_data |= rbuff[7]; // low
    return read_data;
}

// --- pwm
int RS20X30XDeviceIo::getTorqueInSilence( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 1; // Torque in Silence
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, ROM_TORQUE_IN_SILENCE, data_size, rbuff, rsize);
    int read_data = rbuff[7];
    return read_data;
}
int RS20X30XDeviceIo::getWarmUpTime( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 1; // Warm-up Time
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, ROM_WARM_UP_TIME, data_size, rbuff, rsize);
    int read_data = rbuff[7];
    return read_data;
}

// --- margin
int RS20X30XDeviceIo::getCWComplianceMargin( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 1; // CW Compliance Margin
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, ROM_CW_COMPLIANCE_MARGIN, data_size, rbuff, rsize);
    int read_data = rbuff[7];
    return read_data;
}
int RS20X30XDeviceIo::getCCWComplianceMargin( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 1; // CCW Compliance Margin
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, ROM_CCW_COMPLIANCE_MARGIN, data_size, rbuff, rsize);
    int read_data = rbuff[7];
    return read_data;
}
int RS20X30XDeviceIo::getCWComplianceSlope( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 1; // CW Compliance Slope
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, ROM_CW_COMPLIANCE_SLOPE, data_size, rbuff, rsize);
    int read_data = rbuff[7];
    return read_data;
}
int RS20X30XDeviceIo::getCCWComplianceSlope( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 1; // CCW Compliance Slope
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, ROM_CCW_COMPLIANCE_SLOPE, data_size, rbuff, rsize);
    int read_data = rbuff[7];
    return read_data;
}
int RS20X30XDeviceIo::getPunch( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 2; // Punch L + H
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, ROM_PUNCH_L, data_size, rbuff, rsize);
    int read_data = 0;
    read_data = rbuff[8]; // high
    read_data = read_data << 8;
    read_data |= rbuff[7]; // low
    return read_data;
}

// --- --- RAM
// --- ctl
int RS20X30XDeviceIo::getGoalPosition( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 2; // Goal Position L + H
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, RAM_GOAL_POSITION_L, data_size, rbuff, rsize);
    int read_data = 0;
    read_data = rbuff[8]; // high
    read_data = read_data << 8;
    read_data |= rbuff[7]; // low
    return read_data;
}
int RS20X30XDeviceIo::getGoalTime( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 2; // Goal Time L + H
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, RAM_GOAL_TIME_L, data_size, rbuff, rsize);
    int read_data = 0;
    read_data = rbuff[8]; // high
    read_data = read_data << 8;
    read_data |= rbuff[7]; // low
    return read_data;
}
int RS20X30XDeviceIo::getMaxTorque( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 1; // Max Torque
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, RAM_MAX_TORQUE, data_size, rbuff, rsize);
    int read_data = rbuff[7];
    return read_data;
}
int RS20X30XDeviceIo::getTorqueEnable( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 1; // Max Enable
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, RAM_TORQUE_ENABLE, data_size, rbuff, rsize);
    int read_data = rbuff[7];
    return read_data;
}
int RS20X30XDeviceIo::getPIDCoefficient( unsigned char id ){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 1; // PID Coefficient
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];

    int rtn = readPram(id, FLAG_RETURN_SPECIFY, RAM_PID_COEFFICIENT, data_size, rbuff, rsize);
    int read_data = rbuff[7];
    return read_data;
}

// --- status
RS20X30X_STATUS RS20X30XDeviceIo::getStatus(unsigned char id){
    int short_pac_size = 8; // 2(header) + 5(ID,Flg,Adr,Len,Cnt) + 1(Sum)
    int data_size = 18; // 18(No42~59)
    int rsize = short_pac_size + data_size;
    unsigned char rbuff[rsize];
    int rtn = readPram(id, FLAG_RETURN_NO42_59, 0x00, 0x00, rbuff, rsize);

    RS20X30X_STATUS _status;
    _status.present_position       = Buff2Status(RAM_PRESENT_POSITION_L, rbuff) / 10.0; //[deg]
    _status.present_time           = Buff2Status(RAM_PRESENT_TIME_L, rbuff) * 10;       //[ms]
    _status.present_speed          = Buff2Status(RAM_PRESENT_SPEED_L, rbuff);           //[deg/s]
    _status.present_current        = Buff2Status(RAM_PRESENT_CURRENT_L, rbuff);         //[mA]
    _status.present_temperature    = Buff2Status(RAM_PRESENT_TEMPERATURE_L, rbuff);     //[℃]
    _status.present_volts          = Buff2Status(RAM_PRESENT_VOLTS_L, rbuff) / 100.0;    //[V]

    return _status;
}
float RS20X30XDeviceIo::getPosition(unsigned char id){
    RS20X30X_STATUS status = getStatus(id);
    return status.present_position;
}
int RS20X30XDeviceIo::getTime(unsigned char id){
    RS20X30X_STATUS status = getStatus(id);
    return status.present_time;
}
int RS20X30XDeviceIo::getSpeed(unsigned char id){
    RS20X30X_STATUS status = getStatus(id);
    return status.present_speed;
}
int RS20X30XDeviceIo::getCurrent(unsigned char id){
    RS20X30X_STATUS status = getStatus(id);
    return status.present_current;
}
int RS20X30XDeviceIo::getTemperature(unsigned char id){
    RS20X30X_STATUS status = getStatus(id);
    return status.present_temperature;
}
float RS20X30XDeviceIo::getVoltage(unsigned char id){
    RS20X30X_STATUS status = getStatus(id);
    return status.present_volts;
}

/* --- set function --- */
// ROM
// --- info
int RS20X30XDeviceIo::setServoID(unsigned char id, unsigned char new_id ) {
	return writePram(id, FLAG_RETURN_NON, ROM_SERVO_ID, 1, 1, &new_id);
}
int RS20X30XDeviceIo::setReverse(unsigned char id, unsigned char direction ) {
	return writePram(id, FLAG_RETURN_NON, ROM_REVERSE, 1, 1, &direction);
}
int RS20X30XDeviceIo::setReverseCW(unsigned char id) {
    return setReverse(id, REVERSE_CW);
}
int RS20X30XDeviceIo::setReverseCCW(unsigned char id) {
    return setReverse(id, REVERSE_CCW);
}
int RS20X30XDeviceIo::setBaudRate(unsigned char id, unsigned char bps ) {
	return writePram(id, FLAG_RETURN_NON, ROM_BAUDRATE, 1, 1, &bps);
}
int RS20X30XDeviceIo::setBaudRate9600(unsigned char id) {
	return setBaudRate(id, BAUDRATE_9600);
}
int RS20X30XDeviceIo::setBaudRate14400(unsigned char id) {
	return setBaudRate(id, BAUDRATE_14400);
}
int RS20X30XDeviceIo::setBaudRate19200(unsigned char id) {
	return setBaudRate(id, BAUDRATE_19200);
}
int RS20X30XDeviceIo::setBaudRate28800(unsigned char id) {
	return setBaudRate(id, BAUDRATE_28800);
}
int RS20X30XDeviceIo::setBaudRate38400(unsigned char id) {
	return setBaudRate(id, BAUDRATE_38400);
}
int RS20X30XDeviceIo::setBaudRate57600(unsigned char id) {
	return setBaudRate(id, BAUDRATE_57600);
}
int RS20X30XDeviceIo::setBaudRate76800(unsigned char id) {
	return setBaudRate(id, BAUDRATE_76800);
}
int RS20X30XDeviceIo::setBaudRate115200(unsigned char id) {
	return setBaudRate(id, BAUDRATE_115200);
}
int RS20X30XDeviceIo::setBaudRate153600(unsigned char id) {
	return setBaudRate(id, BAUDRATE_153600);
}
int RS20X30XDeviceIo::setBaudRate230400(unsigned char id) {
	return setBaudRate(id, BAUDRATE_230400);
}
int RS20X30XDeviceIo::setReturnDelay(unsigned char id, unsigned int time ) {
    unsigned int default_time = 100; // [us]
    if(time < default_time) time = default_time;
    time = time - default_time;
    time = time / 50;

    unsigned char data[2];
    data[0] = (unsigned char) time & 0xFF;
    data[1] = (unsigned char) (time >> 8) & 0xFF;

	return writePram(id, FLAG_RETURN_NON, ROM_RETURN_DELAY, 2, 1, data);
}
int RS20X30XDeviceIo::setAngleLimit(unsigned char id, short angle) {
    if(angle < MOVE_LIMIT_MIN) angle = MOVE_LIMIT_MIN;
	if(angle > MOVE_LIMIT_MAX) angle = MOVE_LIMIT_MAX;
	angle = angle * 10;

    bool CCW_flag = false;
    if(angle < 0) {
        CCW_flag = true;
        angle = angle * -1;
    }

	unsigned char data[2];
	data[0] = (unsigned char) angle & 0xFF;
	data[1] = (unsigned char) (angle >> 8) & 0xFF;

    if(CCW_flag == true){  // when angle is -, CCW
        return writePram(id, FLAG_RETURN_NON, ROM_CCW_ANGLELIMIT_L, 2, 1, data);
    }
    else{   // when angle is +, CW
        return writePram(id, FLAG_RETURN_NON, ROM_CW_ANGLELIMIT_L, 2, 1, data);
    }
}

// --- pwm
int RS20X30XDeviceIo::setTorqueInSilence(unsigned char id, unsigned char mode) {
	return writePram(id, FLAG_RETURN_NON, ROM_TORQUE_IN_SILENCE, 1, 1, &mode);
}
int RS20X30XDeviceIo::setTorqueInSilenceFree(unsigned char id) {
	return setTorqueInSilence(id, TORQUE_IN_SILENCE_FREE);
}
int RS20X30XDeviceIo::setTorqueInSilenceKeep(unsigned char id) {
	return setTorqueInSilence(id, TORQUE_IN_SILENCE_KEEP);
}
int RS20X30XDeviceIo::setTorqueInSilenceBrake(unsigned char id) {
	return setTorqueInSilence(id, TORQUE_IN_SILENCE_BRAKE);
}
int RS20X30XDeviceIo::setWarmUpTime(unsigned char id, unsigned int time) {
    if(time > MOVE_LIMIT_WARMUPTIME_MAX) time = MOVE_LIMIT_WARMUPTIME_MAX;
    time = time / 10;

    unsigned char data[2];
    data[0] = (unsigned char) time & 0xFF;
    data[1] = (unsigned char) (time >> 8) & 0xFF;

	return writePram(id, FLAG_RETURN_NON, ROM_WARM_UP_TIME, 2, 1, data);
}

// --- compliance
int RS20X30XDeviceIo::setCompliance( unsigned char id, short cw_m_angle, short ccw_m_angle, unsigned char cw_s_angle, unsigned char ccw_s_angle, unsigned char p_percent ) {
    if(cw_m_angle > MOVE_LIMIT_COMPLIANCE_MARGIN) cw_m_angle = MOVE_LIMIT_COMPLIANCE_MARGIN;
    if(ccw_m_angle > MOVE_LIMIT_COMPLIANCE_MARGIN) ccw_m_angle = MOVE_LIMIT_COMPLIANCE_MARGIN;
    if(p_percent > 100) p_percent = 100;

    cw_m_angle = cw_m_angle * 10;
    ccw_m_angle = ccw_m_angle * 10;
    p_percent = p_percent / 100 * 255;

    unsigned char data[6];
    data[0] = (unsigned char) cw_m_angle & 0xFF;
    data[1] = (unsigned char) ccw_m_angle & 0xFF;
    data[2] = (unsigned char) cw_s_angle & 0xFF;
    data[3] = (unsigned char) ccw_s_angle & 0xFF;
    data[4] = (unsigned char) p_percent & 0xFF;
    data[5] = (unsigned char) (p_percent >> 8) & 0xFF;

	return writePram(id, FLAG_RETURN_NON, ROM_CW_COMPLIANCE_MARGIN, 6, 1, data);
}
int RS20X30XDeviceIo::setCWComplianceMargin(unsigned char id, short angle) {
    if(angle > MOVE_LIMIT_COMPLIANCE_MARGIN) angle = MOVE_LIMIT_COMPLIANCE_MARGIN;
    unsigned char data = angle * 10;

	return writePram(id, FLAG_RETURN_NON, ROM_CW_COMPLIANCE_MARGIN, 1, 1, &data);
}
int RS20X30XDeviceIo::setCCWComplianceMargin(unsigned char id, short angle) {
    if(angle > MOVE_LIMIT_COMPLIANCE_MARGIN) angle = MOVE_LIMIT_COMPLIANCE_MARGIN;
    unsigned char data = angle * 10;

	return writePram(id, FLAG_RETURN_NON, ROM_CCW_COMPLIANCE_MARGIN, 1, 1, &data);
}
int RS20X30XDeviceIo::setCCWComplianceSlope(unsigned char id, unsigned char angle) {
	return writePram(id, FLAG_RETURN_NON, ROM_CW_COMPLIANCE_SLOPE, 1, 1, &angle);
}
int RS20X30XDeviceIo::setCWComplianceSlope(unsigned char id, unsigned char angle) {
	return writePram(id, FLAG_RETURN_NON, ROM_CW_COMPLIANCE_SLOPE, 1, 1, &angle);
}
int RS20X30XDeviceIo::setPunch(unsigned char id, unsigned char percent) {
    percent = percent / 100 * 255;
    unsigned char data[2];
    data[0] = (unsigned char) percent & 0xFF;
    data[1] = (unsigned char) (percent >> 8) & 0xFF;
	return writePram(id, FLAG_RETURN_NON, ROM_PUNCH_L, 2, 1, data);
}

// --- goal position
int RS20X30XDeviceIo::setGoalPosition(unsigned char id, short angle) {
	if(angle < MOVE_LIMIT_MIN) angle = MOVE_LIMIT_MIN;
	if(angle > MOVE_LIMIT_MAX) angle = MOVE_LIMIT_MAX;
	angle = angle * 10;

	unsigned char data[2];
	data[0] = (unsigned char) angle & 0xFF;
	data[1] = (unsigned char) (angle >> 8) & 0xFF;

	return writePram(id, FLAG_RETURN_NON, RAM_GOAL_POSITION_L, 2, 1, data);
}

int RS20X30XDeviceIo::setGoalPositionInTime(unsigned char id, short angle, unsigned int time) {
	if(angle < MOVE_LIMIT_MIN) angle = MOVE_LIMIT_MIN;
	if(angle > MOVE_LIMIT_MAX) angle = MOVE_LIMIT_MAX;
	angle = angle * 10;
    time = time / 10;

	unsigned char data[4];
	data[0] = (unsigned char) angle & 0xFF;
	data[1] = (unsigned char) (angle >> 8) & 0xFF;
    data[2] = (unsigned char) time & 0xff;
    data[3] = (unsigned char) (time >> 8) & 0xff;

	return writePram(id, FLAG_RETURN_NON, RAM_GOAL_POSITION_L, 4, 1, data);
}

// --- torque
int RS20X30XDeviceIo::setMaxTorque(unsigned char id, unsigned char percent) {
	return writePram(id, FLAG_RETURN_NON, RAM_MAX_TORQUE, 1, 1, &percent);
}
int RS20X30XDeviceIo::setTorqueMode(unsigned char id, unsigned char mode) {
	return writePram(id, FLAG_RETURN_NON, RAM_TORQUE_ENABLE, 1, 1, &mode);
}
int RS20X30XDeviceIo::setTorqueOff(unsigned char id) {
    return setTorqueMode(id, TORQUE_OFF);
}
int RS20X30XDeviceIo::setTorqueOn(unsigned char id) {
    return setTorqueMode(id, TORQUE_ON);
}
int RS20X30XDeviceIo::setTorqueBrake(unsigned char id) {
    return setTorqueMode(id, TORQUE_BRAKE);
}


// TODO: LongPacket function
#if 0
int RS20X30XDeviceIo::setGoalPositionInTime(short id[], short angle[], short time, int n) {
    int rtn;
    unsigned short int sum;
    unsigned char rf[256];
    int i = 0;

    sum = 0;
    sum = rf[i++] = 3 * n + 6; //size
    sum += rf[i++] = 0x06; //command
    sum += rf[i++] = 0x00; //option
    for (int j = 0; j < n; j++) {
        sum += rf[i++] = id[j];
        sum += rf[i++] = (unsigned char) (pos[j] & 0xff);
        sum += rf[i++] = (unsigned char) ((pos[j] >> 8) & 0xff);
        sum += rf[i++] = (unsigned char) (time & 0xff);
        sum += rf[i++] = (unsigned char) ((time >> 8) & 0xff);
    }
    rf[i++] = (unsigned char) (sum & 0xff);

    rtn = commandSend(rf, 3 * n + 6);

    //    fflush(stdout);
    return rtn;
}
#endif
