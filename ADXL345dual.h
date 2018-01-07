/*
ADXL345dual.h - Header file for the ADXL345 Triple Axis Accelerometer Arduino Library.

Version: 2.1.0
change log: Two ADXL345 devices can be used (Jan 2018), Guray Gurkan
			Just add a new argument as device index being 0 or 1, to each command. Ex: device.begin(0) 

Version: 1.1.0
(c) Korneliusz Jarzebski 
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

#ifndef ADXL345dual_h
#define ADXL345dual_h

#if ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define ADXL345_ADDRESS_0            (0x53) //SDO LOW
#define ADXL345_ADDRESS_1            (0x1D) //SDO HIGH
#define ADXL345_REG_DEVID            (0x00)
#define ADXL345_REG_THRESH_TAP       (0x1D) // 1
#define ADXL345_REG_OFSX             (0x1E)
#define ADXL345_REG_OFSY             (0x1F)
#define ADXL345_REG_OFSZ             (0x20)
#define ADXL345_REG_DUR              (0x21) // 2
#define ADXL345_REG_LATENT           (0x22) // 3
#define ADXL345_REG_WINDOW           (0x23) // 4
#define ADXL345_REG_THRESH_ACT       (0x24) // 5
#define ADXL345_REG_THRESH_INACT     (0x25) // 6
#define ADXL345_REG_TIME_INACT       (0x26) // 7
#define ADXL345_REG_ACT_INACT_CTL    (0x27)
#define ADXL345_REG_THRESH_FF        (0x28) // 8
#define ADXL345_REG_TIME_FF          (0x29) // 9
#define ADXL345_REG_TAP_AXES         (0x2A)
#define ADXL345_REG_ACT_TAP_STATUS   (0x2B)
#define ADXL345_REG_BW_RATE          (0x2C)
#define ADXL345_REG_POWER_CTL        (0x2D)
#define ADXL345_REG_INT_ENABLE       (0x2E)
#define ADXL345_REG_INT_MAP          (0x2F)
#define ADXL345_REG_INT_SOURCE       (0x30) // A
#define ADXL345_REG_DATA_FORMAT      (0x31)
#define ADXL345_REG_DATAX0           (0x32)
#define ADXL345_REG_DATAX1           (0x33)
#define ADXL345_REG_DATAY0           (0x34)
#define ADXL345_REG_DATAY1           (0x35)
#define ADXL345_REG_DATAZ0           (0x36)
#define ADXL345_REG_DATAZ1           (0x37)
#define ADXL345_REG_FIFO_CTL         (0x38)
#define ADXL345_REG_FIFO_STATUS      (0x39)

#define ADXL345_GRAVITY_SUN          273.95f
#define ADXL345_GRAVITY_EARTH        9.80665f
#define ADXL345_GRAVITY_MOON         1.622f
#define ADXL345_GRAVITY_MARS         3.69f
#define ADXL345_GRAVITY_NONE         1.00f

typedef enum
{
    ADXL345_DATARATE_3200HZ    = 0b1111,
    ADXL345_DATARATE_1600HZ    = 0b1110,
    ADXL345_DATARATE_800HZ     = 0b1101,
    ADXL345_DATARATE_400HZ     = 0b1100,
    ADXL345_DATARATE_200HZ     = 0b1011,
    ADXL345_DATARATE_100HZ     = 0b1010,
    ADXL345_DATARATE_50HZ      = 0b1001,
    ADXL345_DATARATE_25HZ      = 0b1000,
    ADXL345_DATARATE_12_5HZ    = 0b0111,
    ADXL345_DATARATE_6_25HZ    = 0b0110,
    ADXL345_DATARATE_3_13HZ    = 0b0101,
    ADXL345_DATARATE_1_56HZ    = 0b0100,
    ADXL345_DATARATE_0_78HZ    = 0b0011,
    ADXL345_DATARATE_0_39HZ    = 0b0010,
    ADXL345_DATARATE_0_20HZ    = 0b0001,
    ADXL345_DATARATE_0_10HZ    = 0b0000
} adxl345_dataRate_t;

typedef enum
{
    ADXL345_INT2 = 0b01,
    ADXL345_INT1 = 0b00
} adxl345_int_t;

typedef enum
{
    ADXL345_DATA_READY         = 0x07,
    ADXL345_SINGLE_TAP         = 0x06,
    ADXL345_DOUBLE_TAP         = 0x05,
    ADXL345_ACTIVITY           = 0x04,
    ADXL345_INACTIVITY         = 0x03,
    ADXL345_FREE_FALL          = 0x02,
    ADXL345_WATERMARK          = 0x01,
    ADXL345_OVERRUN            = 0x00
} adxl345_activity_t;

typedef enum
{
    ADXL345_RANGE_16G          = 0b11,
    ADXL345_RANGE_8G           = 0b10,
    ADXL345_RANGE_4G           = 0b01,
    ADXL345_RANGE_2G           = 0b00
} adxl345_range_t;

#ifndef VECTOR_STRUCT_H
#define VECTOR_STRUCT_H
struct Vector
{
    float XAxis;
    float YAxis;
    float ZAxis;
};
#endif

struct Activites
{
    bool isOverrun;
    bool isWatermark;
    bool isFreeFall;
    bool isInactivity;
    bool isActivity;
    bool isActivityOnX;
    bool isActivityOnY;
    bool isActivityOnZ;
    bool isDoubleTap;
    bool isTap;
    bool isTapOnX;
    bool isTapOnY;
    bool isTapOnZ;
    bool isDataReady;
};

class ADXL345
{
    public:

	bool begin(int index);
	void clearSettings(int index);

	Vector readRaw(int index);
	Vector readNormalize(int index,float gravityFactor = ADXL345_GRAVITY_EARTH);
	Vector readScaled(int index);

	Activites readActivites(int index);

	Vector lowPassFilter(int index,Vector vector, float alpha = 0.5);

	void  setRange(int index,adxl345_range_t range);
	adxl345_range_t getRange(int index);

	void  setDataRate(int index,adxl345_dataRate_t dataRate);
	adxl345_dataRate_t getDataRate(int index);

	void setTapThreshold(int index,float threshold);
	float getTapThreshold(int index);

	void setTapDuration(int index,float duration);
	float getTapDuration(int index);

	void setDoubleTapLatency(int index,float latency);
	float getDoubleTapLatency(int index);

	void setDoubleTapWindow(int index, float window);
	float getDoubleTapWindow(int index);

	void setActivityThreshold(int index, float threshold);
	float getActivityThreshold(int index);

	void setInactivityThreshold(int index,float threshold);
	float getInactivityThreshold(int index);

	void setTimeInactivity(int index,uint8_t time);
	uint8_t getTimeInactivity(int index);

	void setFreeFallThreshold(int index, float threshold);
	float getFreeFallThreshold(int index);

	void setFreeFallDuration(int index, float duration);
	float getFreeFallDuration(int index);

	void setActivityX(int index, bool state);
	bool getActivityX(int index);
	void setActivityY(int index, bool state);
	bool getActivityY(int index);
	void setActivityZ(int index, bool state);
	bool getActivityZ(int index);
	void setActivityXYZ(int index, bool state);

	void setInactivityX(int index, bool state);
	bool getInactivityX(int index);
	void setInactivityY(int index, bool state);
	bool getInactivityY(int index);
	void setInactivityZ(int index, bool state);
	bool getInactivityZ(int index);
	void setInactivityXYZ(int index, bool state);

	void setTapDetectionX(int index, bool state);
	bool getTapDetectionX(int index);
	void setTapDetectionY(int index, bool state);
	bool getTapDetectionY(int index);
	void setTapDetectionZ(int index, bool state);
	bool getTapDetectionZ(int index);
	void setTapDetectionXYZ(int index, bool state);

	void useInterrupt(int index,adxl345_int_t interrupt);


    private:
	Vector r;
	Vector n;
	Vector f;
	Activites a;
	adxl345_range_t _range;

	void writeRegister8(int index,uint8_t reg, uint8_t value);
	uint8_t readRegister8(int index,uint8_t reg);
	uint8_t fastRegister8(int index,uint8_t reg);
	int16_t readRegister16(int index,uint8_t reg);
	void writeRegisterBit(int index,uint8_t reg, uint8_t pos, bool state);
	bool readRegisterBit(int index,uint8_t reg, uint8_t pos);


};

#endif