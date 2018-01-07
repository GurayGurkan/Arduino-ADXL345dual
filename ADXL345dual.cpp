/*
ADXL345_dual.cpp - Class file for the ADXL345 Triple Axis Accelerometer Arduino Library.

Version: 1.1.0
(c) 2014 Korneliusz Jarzebski
www.jarzebski.pl

This program is free software: you can redistribute it and/or modify
it under the terms of the version 3 GNU General Public License as
published by the Free Software Foundation.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#include <Wire.h>

#include "ADXL345dual.h"

bool ADXL345::begin(int index)
{
    f.XAxis = 0;
    f.YAxis = 0;
    f.ZAxis = 0;

    Wire.begin();

    // Check ADXL345 REG DEVID
    if (fastRegister8(index, ADXL345_REG_DEVID) != 0xE5)
    {
        return false;
    }

    // Enable measurement mode (0b00001000)
    writeRegister8(index, ADXL345_REG_POWER_CTL, 0x08);

    clearSettings(index);

    return true;
}

// Set Range
void ADXL345::setRange(int index, adxl345_range_t range)
{
  // Get actual value register
  uint8_t value = readRegister8(index, ADXL345_REG_DATA_FORMAT);

  // Update the data rate
  // (&) 0b11110000 (0xF0 - Leave HSB)
  // (|) 0b0000xx?? (range - Set range)
  // (|) 0b00001000 (0x08 - Set Full Res)
  value &= 0xF0;
  value |= range;
  value |= 0x08;

  writeRegister8(index, ADXL345_REG_DATA_FORMAT, value);
}

// Get Range
adxl345_range_t ADXL345::getRange(int index)
{
    return (adxl345_range_t)(readRegister8(index, ADXL345_REG_DATA_FORMAT) & 0x03);
}

// Set Data Rate
void ADXL345::setDataRate(int index, adxl345_dataRate_t dataRate)
{
    writeRegister8(index, ADXL345_REG_BW_RATE, dataRate);
}

// Get Data Rate
adxl345_dataRate_t ADXL345::getDataRate(int index)
{
    return (adxl345_dataRate_t)(readRegister8(index, ADXL345_REG_BW_RATE) & 0x0F);
}

// Low Pass Filter
Vector ADXL345::lowPassFilter(int index, Vector vector, float alpha)
{
    f.XAxis = vector.XAxis * alpha + (f.XAxis * (1.0 - alpha));
    f.YAxis = vector.YAxis * alpha + (f.YAxis * (1.0 - alpha));
    f.ZAxis = vector.ZAxis * alpha + (f.ZAxis * (1.0 - alpha));
    return f;
}

// Read raw values
Vector ADXL345::readRaw(int index)
{
    r.XAxis = readRegister16(index, ADXL345_REG_DATAX0);
    r.YAxis = readRegister16(index, ADXL345_REG_DATAY0);
    r.ZAxis = readRegister16(index, ADXL345_REG_DATAZ0);
    return r;
}

// Read normalized values
Vector ADXL345::readNormalize(int index, float gravityFactor)
{
    readRaw(index);

    // (4 mg/LSB scale factor in Full Res) * gravity factor
    n.XAxis = r.XAxis * 0.004 * gravityFactor;
    n.YAxis = r.YAxis * 0.004 * gravityFactor;
    n.ZAxis = r.ZAxis * 0.004 * gravityFactor;

    return n;
}

// Read scaled values
Vector ADXL345::readScaled(int index)
{
    readRaw(index);

    // (4 mg/LSB scale factor in Full Res)
    n.XAxis = r.XAxis * 0.004;
    n.YAxis = r.YAxis * 0.004;
    n.ZAxis = r.ZAxis * 0.004;

    return n;
}

void ADXL345::clearSettings(int index)
{
    setRange(index, ADXL345_RANGE_2G);
    setDataRate(index, ADXL345_DATARATE_100HZ);

    writeRegister8(index, ADXL345_REG_THRESH_TAP, 0x00);
    writeRegister8(index, ADXL345_REG_DUR, 0x00);
    writeRegister8(index, ADXL345_REG_LATENT, 0x00);
    writeRegister8(index, ADXL345_REG_WINDOW, 0x00);
    writeRegister8(index, ADXL345_REG_THRESH_ACT, 0x00);
    writeRegister8(index, ADXL345_REG_THRESH_INACT, 0x00);
    writeRegister8(index, ADXL345_REG_TIME_INACT, 0x00);
    writeRegister8(index, ADXL345_REG_THRESH_FF, 0x00);
    writeRegister8(index, ADXL345_REG_TIME_FF, 0x00);

    uint8_t value;

    value = readRegister8(index, ADXL345_REG_ACT_INACT_CTL);
    value &= 0b10001000;
    writeRegister8(index, ADXL345_REG_ACT_INACT_CTL, value);

    value = readRegister8(index, ADXL345_REG_TAP_AXES);
    value &= 0b11111000;
    writeRegister8(index, ADXL345_REG_TAP_AXES, value);
}

// Set Tap Threshold (62.5mg / LSB)
void ADXL345::setTapThreshold(int index, float threshold)
{
    uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
    writeRegister8(index, ADXL345_REG_THRESH_TAP, scaled);
}

// Get Tap Threshold (62.5mg / LSB)
float ADXL345::getTapThreshold(int index)
{
    return readRegister8(index, ADXL345_REG_THRESH_TAP) * 0.0625f;
}

// Set Tap Duration (625us / LSB)
void ADXL345::setTapDuration(int index, float duration)
{
    uint8_t scaled = constrain(duration / 0.000625f, 0, 255);
    writeRegister8(index, ADXL345_REG_DUR, scaled);
}

// Get Tap Duration (625us / LSB)
float ADXL345::getTapDuration(int index)
{
    return readRegister8(index, ADXL345_REG_DUR) * 0.000625f;
}

// Set Double Tap Latency (1.25ms / LSB)
void ADXL345::setDoubleTapLatency(int index, float latency)
{
    uint8_t scaled = constrain(latency / 0.00125f, 0, 255);
    writeRegister8(index, ADXL345_REG_LATENT, scaled);
}

// Get Double Tap Latency (1.25ms / LSB)
float ADXL345::getDoubleTapLatency(int index)
{
    return readRegister8(index, ADXL345_REG_LATENT) * 0.00125f;
}

// Set Double Tap Window (1.25ms / LSB)
void ADXL345::setDoubleTapWindow(int index, float window)
{
    uint8_t scaled = constrain(window / 0.00125f, 0, 255);
    writeRegister8(index, ADXL345_REG_WINDOW, scaled);
}

// Get Double Tap Window (1.25ms / LSB)
float ADXL345::getDoubleTapWindow(int index)
{
    return readRegister8(index, ADXL345_REG_WINDOW) * 0.00125f;
}

// Set Activity Threshold (62.5mg / LSB)
void ADXL345::setActivityThreshold(int index, float threshold)
{
    uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
    writeRegister8(index, ADXL345_REG_THRESH_ACT, scaled);
}

// Get Activity Threshold (65.5mg / LSB)
float ADXL345::getActivityThreshold(int index)
{
    return readRegister8(index, ADXL345_REG_THRESH_ACT) * 0.0625f;
}

// Set Inactivity Threshold (65.5mg / LSB)
void ADXL345::setInactivityThreshold(int index, float threshold)
{
    uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
    writeRegister8(index, ADXL345_REG_THRESH_INACT, scaled);
}

// Get Incactivity Threshold (65.5mg / LSB)
float ADXL345::getInactivityThreshold(int index)
{
    return readRegister8(index, ADXL345_REG_THRESH_INACT) * 0.0625f;
}

// Set Inactivity Time (s / LSB)
void ADXL345::setTimeInactivity(int index, uint8_t time)
{
    writeRegister8(index, ADXL345_REG_TIME_INACT, time);
}

// Get Inactivity Time (s / LSB)
uint8_t ADXL345::getTimeInactivity(int index)
{
    return readRegister8(index, ADXL345_REG_TIME_INACT);
}

// Set Free Fall Threshold (65.5mg / LSB)
void ADXL345::setFreeFallThreshold(int index, float threshold)
{
    uint8_t scaled = constrain(threshold / 0.0625f, 0, 255);
    writeRegister8(index, ADXL345_REG_THRESH_FF, scaled);
}

// Get Free Fall Threshold (65.5mg / LSB)
float ADXL345::getFreeFallThreshold(int index)
{
    return readRegister8(index, ADXL345_REG_THRESH_FF) * 0.0625f;
}

// Set Free Fall Duratiom (5ms / LSB)
void ADXL345::setFreeFallDuration(int index, float duration)
{
    uint8_t scaled = constrain(duration / 0.005f, 0, 255);
    writeRegister8(index, ADXL345_REG_TIME_FF, scaled);
}

// Get Free Fall Duratiom
float ADXL345::getFreeFallDuration(int index)
{
    return readRegister8(index, ADXL345_REG_TIME_FF) * 0.005f;
}

void ADXL345::setActivityX(int index, bool state)
{
    writeRegisterBit(index, ADXL345_REG_ACT_INACT_CTL, 6, state);
}

bool ADXL345::getActivityX(int index)
{
    return readRegisterBit(index, ADXL345_REG_ACT_INACT_CTL, 6);
}

void ADXL345::setActivityY(int index, bool state)
{
    writeRegisterBit(index, ADXL345_REG_ACT_INACT_CTL, 5, state);
}

bool ADXL345::getActivityY(int index)
{
    return readRegisterBit(index, ADXL345_REG_ACT_INACT_CTL, 5);
}

void ADXL345::setActivityZ(int index, bool state)
{
    writeRegisterBit(index, ADXL345_REG_ACT_INACT_CTL, 4, state);
}

bool ADXL345::getActivityZ(int index)
{
    return readRegisterBit(index, ADXL345_REG_ACT_INACT_CTL, 4);
}

void ADXL345::setActivityXYZ(int index, bool state)
{
    uint8_t value;

    value = readRegister8(index, ADXL345_REG_ACT_INACT_CTL);

    if (state)
    {
	value |= 0b00111000;
    } else
    {
	value &= 0b11000111;
    }

    writeRegister8(index, ADXL345_REG_ACT_INACT_CTL, value);
}


void ADXL345::setInactivityX(int index, bool state) 
{
    writeRegisterBit(index, ADXL345_REG_ACT_INACT_CTL, 2, state);
}

bool ADXL345::getInactivityX(int index)
{
    return readRegisterBit(index, ADXL345_REG_ACT_INACT_CTL, 2);
}

void ADXL345::setInactivityY(int index, bool state)
{
    writeRegisterBit(index, ADXL345_REG_ACT_INACT_CTL, 1, state);
}

bool ADXL345::getInactivityY(int index)
{
    return readRegisterBit(index, ADXL345_REG_ACT_INACT_CTL, 1);
}

void ADXL345::setInactivityZ(int index, bool state)
{
    writeRegisterBit(index, ADXL345_REG_ACT_INACT_CTL, 0, state);
}

bool ADXL345::getInactivityZ(int index)
{
    return readRegisterBit(index, ADXL345_REG_ACT_INACT_CTL, 0);
}

void ADXL345::setInactivityXYZ(int index, bool state)
{
    uint8_t value;

    value = readRegister8(index, ADXL345_REG_ACT_INACT_CTL);

    if (state)
    {
	value |= 0b00000111;
    } else
    {
	value &= 0b11111000;
    }

    writeRegister8(index, ADXL345_REG_ACT_INACT_CTL, value);
}

void ADXL345::setTapDetectionX(int index, bool state)
{
    writeRegisterBit(index, ADXL345_REG_TAP_AXES, 2, state);
}

bool ADXL345::getTapDetectionX(int index)
{
    return readRegisterBit(index, ADXL345_REG_TAP_AXES, 2);
}

void ADXL345::setTapDetectionY(int index, bool state)
{
    writeRegisterBit(index, ADXL345_REG_TAP_AXES, 1, state);
}

bool ADXL345::getTapDetectionY(int index)
{
    return readRegisterBit(index, ADXL345_REG_TAP_AXES, 1);
}

void ADXL345::setTapDetectionZ(int index, bool state)
{
    writeRegisterBit(index, ADXL345_REG_TAP_AXES, 0, state);
}

bool ADXL345::getTapDetectionZ(int index)
{
    return readRegisterBit(index, ADXL345_REG_TAP_AXES, 0);
}

void ADXL345::setTapDetectionXYZ(int index, bool state)
{
    uint8_t value;

    value = readRegister8(index, ADXL345_REG_TAP_AXES);

    if (state)
    {
	value |= 0b00000111;
    } else
    {
	value &= 0b11111000;
    }

    writeRegister8(index, ADXL345_REG_TAP_AXES, value);
}


void ADXL345::useInterrupt(int index, adxl345_int_t interrupt)
{
    if (interrupt == 0)
    {
	writeRegister8(index, ADXL345_REG_INT_MAP, 0x00);
    } else
    {
	writeRegister8(index, ADXL345_REG_INT_MAP, 0xFF);
    }

    writeRegister8(index, ADXL345_REG_INT_ENABLE, 0xFF);
}

Activites ADXL345::readActivites(int index)
{
    uint8_t data = readRegister8(index, ADXL345_REG_INT_SOURCE);

    a.isOverrun = ((data >> ADXL345_OVERRUN) & 1);
    a.isWatermark = ((data >> ADXL345_WATERMARK) & 1);
    a.isFreeFall = ((data >> ADXL345_FREE_FALL) & 1);
    a.isInactivity = ((data >> ADXL345_INACTIVITY) & 1);
    a.isActivity = ((data >> ADXL345_ACTIVITY) & 1);
    a.isDoubleTap = ((data >> ADXL345_DOUBLE_TAP) & 1);
    a.isTap = ((data >> ADXL345_SINGLE_TAP) & 1);
    a.isDataReady = ((data >> ADXL345_DATA_READY) & 1);

    data = readRegister8(index, ADXL345_REG_ACT_TAP_STATUS);

    a.isActivityOnX = ((data >> 6) & 1);
    a.isActivityOnY = ((data >> 5) & 1);
    a.isActivityOnZ = ((data >> 4) & 1);
    a.isTapOnX = ((data >> 2) & 1);
    a.isTapOnY = ((data >> 1) & 1);
    a.isTapOnZ = ((data >> 0) & 1);

    return a;
}

// Write byte to register
void ADXL345::writeRegister8(int index, uint8_t reg, uint8_t value)
{
	if (index==0)
		Wire.beginTransmission(ADXL345_ADDRESS_0);
	else
		Wire.beginTransmission(ADXL345_ADDRESS_1);
    #if ARDUINO >= 100
        Wire.write(reg);
        Wire.write(value);
    #else
        Wire.send(reg);
        Wire.send(value);
    #endif
    Wire.endTransmission();
}

// Read byte to register
uint8_t ADXL345::fastRegister8(int index, uint8_t reg)
{
    uint8_t value;
	if (index==0)
		Wire.beginTransmission(ADXL345_ADDRESS_0);
	else
		Wire.beginTransmission(ADXL345_ADDRESS_1);

    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();
	
	if (index==0)
		Wire.requestFrom(ADXL345_ADDRESS_0,1);
	else
		Wire.requestFrom(ADXL345_ADDRESS_1,1);

    
    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif;
    Wire.endTransmission();

    return value;
}

// Read byte from register
uint8_t ADXL345::readRegister8(int index, uint8_t reg)
{
    uint8_t value;
	if (index==0)
		Wire.beginTransmission(ADXL345_ADDRESS_0);
	else
		Wire.beginTransmission(ADXL345_ADDRESS_1);

    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();
	
	if (index==0)
	{
		Wire.beginTransmission(ADXL345_ADDRESS_0);
		Wire.requestFrom(ADXL345_ADDRESS_0,1);
	}
	else
	{
		Wire.beginTransmission(ADXL345_ADDRESS_1);
		Wire.requestFrom(ADXL345_ADDRESS_1,1);
	}


    while(!Wire.available()) {};
    #if ARDUINO >= 100
        value = Wire.read();
    #else
        value = Wire.receive();
    #endif;
    Wire.endTransmission();

    return value;
}

// Read word from register
int16_t ADXL345::readRegister16(int index, uint8_t reg)
{
    int16_t value;
    if (index==0)
		Wire.beginTransmission(ADXL345_ADDRESS_0);
	else
		Wire.beginTransmission(ADXL345_ADDRESS_1);
    #if ARDUINO >= 100
        Wire.write(reg);
    #else
        Wire.send(reg);
    #endif
    Wire.endTransmission();
	
	if (index==0)
	{
		Wire.beginTransmission(ADXL345_ADDRESS_0);
		Wire.requestFrom(ADXL345_ADDRESS_0,2);
	}
	else
	{
		Wire.beginTransmission(ADXL345_ADDRESS_1);
		Wire.requestFrom(ADXL345_ADDRESS_1,2);
	}

    while(!Wire.available()) {};
    #if ARDUINO >= 100
        uint8_t vla = Wire.read();
        uint8_t vha = Wire.read();
    #else
        uint8_t vla = Wire.receive();
        uint8_t vha = Wire.receive();
    #endif;
    Wire.endTransmission();

    value = vha << 8 | vla;

    return value;
}

void ADXL345::writeRegisterBit(int index, uint8_t reg, uint8_t pos, bool state)
{
    uint8_t value;
    value = readRegister8(index, reg);

    if (state)
    {
	value |= (1 << pos);
    } else 
    {
	value &= ~(1 << pos);
    }

    writeRegister8(index, reg, value);
}

bool ADXL345::readRegisterBit(int index, uint8_t reg, uint8_t pos)
{
    uint8_t value;
    value = readRegister8(index, reg);
    return ((value >> pos) & 1);
}
