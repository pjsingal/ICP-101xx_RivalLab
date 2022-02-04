//1. send commmand to ICP to begin measure. Save timestamp
//2. 1.8 ms for ICP to process message and take measurements (ICP to board sends 2 8-bit data packets)
//3. board sends request for data from ICP (400 kHz of bits sent from ICP to board. Some of this includes protocol AKA junk information)
//4. board stores the data (read the first 3 8-bit packs). So the ICP does two transmits which each have 2 8-bit packets. We ignore the 4th pacet bc temp
//5. do a "pull down" i.e., stop the transmit and tell ICP to not send any more info
//6. Use SPI protocol to store data on RAM (if there is no memory storage on board. We'll fix this later. Look into what the fastest SD card we could get it. It should be as big as possible 

// Bonus: do some timing exercises, look up code optimization strategies

#include <Arduino.h>
#include <Wire.h> //We might not use this lib
#include "icp101xx.h"

#define ICP_I2C_ID 0x63

#define ICP_CMD_READ_ID 0xefc8
#define ICP_CMD_SET_ADDR 0xc595
#define ICP_CMD_READ_OTP 0xc7f7
#define ICP_CMD_MEAS_LP 0x609c
#define ICP_CMD_MEAS_N 0x6825
#define ICP_CMD_MEAS_LN 0x70df
#define ICP_CMD_MEAS_ULN 0x7866

bool ICP101xx::begin(TwoWire* wire) {
	// setup I2C object
	if (wire) {
		_i2c = wire;
	} else {
		_i2c = &Wire;
	}
	_i2c->begin();

	// verify that the sensor is repsonding
	if (!isConnected()) {
		return false;
	}

	// read sensor calibration data
	uint8_t addr_otp_cmd[5] = { 
				(ICP_CMD_SET_ADDR >> 8) & 0xff,
				ICP_CMD_SET_ADDR & 0xff,
				0x00, 0x66, 0x9c };
	uint8_t otp_buf[3];
	_sendCommand(addr_otp_cmd, 5);
	for (int i=0; i<4; i++) {
		_sendCommand(ICP_CMD_READ_OTP);
		_readResponse(otp_buf, 3);
		_scal[i] = (otp_buf[0] << 8) | otp_buf[1];
	}
	return true;
}

bool ICP101xx::isConnected(void) {
	uint8_t id_buf[2];
	_sendCommand(ICP_CMD_READ_ID);
	_readResponse(id_buf, 2);
	uint16_t id = (id_buf[0] << 8) | id_buf[1];
	if ((id & 0x03f) == 0x08) {
		return true;
	} else {
		return false;
	}
}

void ICP101xx::measure(ICP101xx::mmode mode) { // define mode = ICP101xx::FAST
	cmd = ICP_CMD_MEAS_LP;
	_sendCommand(cmd);
	// look into lib and find whatever timestamp fxn they use and add here
	delay(measureStart(mode)); // don't need to have ICP101xx:: in front bc member fxn is within the class already
	while (!dataReady());
}

bool ICP101xx::dataReady(void) {
	if (_data_ready)
		return true;

	_data_ready = true;
	return true;
}

float ICP101xx::getPressurePa(void) {
	return _pressure_Pa;
}

void ICP101xx::_calculate(void) {
	// calculate pressure
	float t = (float)(_raw_t - 32768);
	float s1 = _lut_lower + (float)(_scal[0] * t * t) * _quadr_factor;
	float s2 = _offst_factor * _scal[3] + (float)(_scal[1] * t * t) * _quadr_factor;
	float s3 = _lut_upper + (float)(_scal[2] * t * t) * _quadr_factor;
	float c = (s1 * s2 * (_pcal[0] - _pcal[1]) +
			   s2 * s3 * (_pcal[1] - _pcal[2]) +
			   s3 * s1 * (_pcal[2] - _pcal[0])) /
			  (s3 * (_pcal[0] - _pcal[1]) +
			   s1 * (_pcal[1] - _pcal[2]) +
			   s2 * (_pcal[2] - _pcal[0]));
	float a = (_pcal[0] * s1 - _pcal[1] * s2 - (_pcal[1] - _pcal[0]) * c) / (s1 - s2);
	float b = (_pcal[0] - a) * (s1 + c);
	_pressure_Pa = a + b / (c + _raw_p);
}

void ICP101xx::_sendCommand(uint16_t cmd) {
	if (!_i2c) return;
	_i2c->beginTransmission(ICP_I2C_ID);
	_i2c->write((cmd >> 8) & 0xff);
	_i2c->write(cmd & 0xff);
	_i2c->endTransmission();
}

void ICP101xx::_sendCommand(uint8_t *cmd_buf, uint8_t cmd_len) {
	if (!_i2c) return;
	if (cmd_buf && cmd_len) {
		_i2c->beginTransmission(ICP_I2C_ID);
		for (int i=0; i<cmd_len; i++) {
			_i2c->write(cmd_buf[i]);
		}
		_i2c->endTransmission();
	}
}

void ICP101xx::_readResponse(uint8_t *res_buf, uint8_t res_len) {
	if (!_i2c) return;
	if (res_buf && res_len) {
		_i2c->requestFrom(ICP_I2C_ID, res_len);
		for (int i=0; i<res_len; i++) {
			res_buf[i] = _i2c->read();
		}
	}
}
