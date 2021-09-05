/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  suppport for serial connected AHRS systems
 */

#pragma once

#include "AP_ExternalAHRS_backend.h"

#if HAL_EXTERNAL_AHRS_ENABLED

#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_ExternalAHRS_WIT : public AP_ExternalAHRS_backend {

public:
    AP_ExternalAHRS_WIT(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(int8_t *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(mavlink_channel_t chan) const override;

    // check for new data
    void update() override {
        check_uart();
    }

private:
    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    bool port_opened;
    uint32_t baudrate;
    uint16_t rate;

    void update_thread();
    bool check_uart();

    #define SAVE 			0x00
    #define CALSW 		0x01
    #define RSW 			0x02
    #define RRATE			0x03
    #define BAUD 			0x04
    #define AXOFFSET	0x05
    #define AYOFFSET	0x06
    #define AZOFFSET	0x07
    #define GXOFFSET	0x08
    #define GYOFFSET	0x09
    #define GZOFFSET	0x0a
    #define HXOFFSET	0x0b
    #define HYOFFSET	0x0c
    #define HZOFFSET	0x0d

    #define YYMM			0x30
    #define DDHH			0x31
    #define MMSS			0x32
    #define MS				0x33
    #define AX				0x34
    #define AY				0x35
    #define AZ				0x36
    #define GX				0x37
    #define GY				0x38
    #define GZ				0x39
    #define HX				0x3a
    #define HY				0x3b
    #define HZ				0x3c
    #define Roll			0x3d
    #define Pitch			0x3e
    #define Yaw				0x3f
    #define TEMP			0x40

    struct STime
    {
    	uint8_t ucYear;
    	uint8_t ucMonth;
    	uint8_t ucDay;
    	uint8_t ucHour;
    	uint8_t ucMinute;
    	uint8_t ucSecond;
    	uint16_t usMiliSecond;
    };
    struct SAcc
    {
    	int16_t a[3]; //x,y,z
    	int16_t T; //timestamp
    };
    struct SGyro
    {
    	int16_t w[3]; //x,y,z
    	int16_t T; //timestamp
    };
    struct SAngle
    {
    	int16_t Angle[3]; //x,y,z
    	int16_t T; //timestamp
    };
    struct SMag
    {
    	int16_t h[3]; //x,y,z
    	int16_t T; //timestamp
    };

    struct SQuater
    {
    	int16_t q0;
    	int16_t q1;
    	int16_t q2;
    	int16_t q3;
    };

    class WITMotion
    {
      public:
    	struct STime		stcTime;
    	struct SAcc 		stcAcc;
    	struct SGyro 		stcGyro;
    	struct SAngle 	stcAngle;
    	struct SMag 		stcMag;
    	struct SQuater	stcQuater;

      WITMotion ();
      void CopeSerialData(uint8_t ucData);
    	int16_t ReadWord(uint8_t ucAddr);
    	void WriteWord(uint8_t ucAddr,int16_t sData);
    	void ReadData(uint8_t ucAddr,uint8_t ucLength,int8_t chrData[]);
    	void GetTime();
    	void GetAcc();
    	void GetGyro();
    	void GetAngle();
    	void GetMag();

      private:
    	uint8_t ucDevAddr;
    	void readRegisters(uint8_t deviceAddr,uint8_t addressToRead, uint8_t bytesToRead, int8_t * dest);
    	void writeRegister(uint8_t deviceAddr,uint8_t addressToWrite,uint8_t bytesToRead, int8_t *dataToWrite);
    };


#endif  // HAL_EXTERNAL_AHRS_ENABLED
