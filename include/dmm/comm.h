#ifndef DMM_H
#define DMM_H

#include "serial\serial.h"

namespace DMM {

	serial::Serial cSerial;

	void sendPackage(unsigned char packageLength, unsigned char B[8]);
	void readPackages(serial::Serial& cSerial);
	void interpretPackage(unsigned char packageID, unsigned char packageLength, unsigned char B[8]);
	unsigned char calcCRC(unsigned char packageLength, unsigned char B[8]);
	long calcSignedValue(unsigned char packageLength, unsigned char B[8]);
	
	
	/* Drive input functions */
	void setOrigin(unsigned char ID);
	void goAbsolutePos(unsigned char ID, int pos);
	void makeLinearLine(unsigned char ID, int line);
	void goRelativePos(unsigned char ID, int pos);
	void makeCircularArc(unsigned char ID, int pos);
	void assignDriveID(unsigned char ID, unsigned char id);
	void readDriveID(unsigned char ID);
	void setDriveConfig(unsigned char ID, unsigned char cmdMode, unsigned char moveMode, unsigned char servoMode, unsigned char active);
	void readDriveConfig(unsigned char ID);
	void readDriveStatus(unsigned char ID);
	void turnConstSpeed(unsigned char ID, int speed);
	void squareWave(unsigned char ID, int amplitude);
	void sinWave(unsigned char ID, int amplitude);
	void setFrequency(unsigned char ID, int frequency);
	void readPosition(unsigned char ID);
	void readTorqueCurrent(unsigned char ID);
	void setMainGain(unsigned char ID, unsigned char gain);
	void setSpeedGain(unsigned char ID, unsigned char gain);
	void setIntGain(unsigned char ID, unsigned char gain);
	void setTorqueCons(unsigned char ID, unsigned char constant);
	void setSpeedLimit(unsigned char ID, unsigned char speed);
	void SetAccelLimit(unsigned char ID, unsigned char accel);
	void setPosOnRange(unsigned char ID, unsigned char range);
	void setGearNumber(unsigned char ID, unsigned short gearN);
	void readMainGain(unsigned char ID);
	void readSpeedGain(unsigned char ID);
	void readIntGain(unsigned char ID);
	void readTorqueCons(unsigned char ID);
	void readSpeedLimit(unsigned char ID);
	void readAccelLimit(unsigned char ID);
	void readPosOnRange(unsigned char ID);
	void readGearNumber(unsigned char ID);

	/* Drive output functions */
	void isMainGain(unsigned char mID, unsigned char B[8]);
	void isSpeedGain(unsigned char mID, unsigned char B[8]);
	void isIntGain(unsigned char mID, unsigned char B[8]);
	void isTorqueCons(unsigned char mID, unsigned char B[8]);
	void isSpeedLimit(unsigned char mID, unsigned char B[8]);
	void isAccelLimit(unsigned char mID, unsigned char B[8]);
	void isDriveID(unsigned char mID, unsigned char B[8]);
	void isPosOnRange(unsigned char mID, unsigned char B[8]);
	void isGearNumber(unsigned char mID, unsigned char B[8]);
	void isStatus(unsigned char mID, unsigned char B[8]);
	void isConfig(unsigned char mID, unsigned char B[8]);
	void isAbsPosition(unsigned char mID, unsigned char B[8]);
	void isTorqueCurrent(unsigned char mID, unsigned char B[8]);
}


#define StartBit 0x7f
#define NotStartBit 0x80

/* Drive function codes */
#define Set_Origin 0x00
#define Go_Absolute_Pos 0x01
#define Make_LinearLine 0x02
#define Go_Relative_Pos 0x03
#define Make_CircularArc 0x04
#define Assign_Drive_ID 0x05
#define Read_Drive_ID 0x06
#define Set_Drive_Config 0x07
#define Read_Drive_Config 0x08
#define Read_Drive_Status 0x09
#define Turn_ConstSpeed 0x0a
#define Square_Wave 0x0b
#define Sin_Wave 0x0c
#define SS_Frequency 0x0d
#define General_Read 0x0e
#define ForMotorDefine 0x0f
#define Set_MainGain 0x10
#define Set_SpeedGain 0x11
#define Set_IntGain 0x12
#define Set_TrqCons 0x13
#define Set_HighSpeed 0x14
#define Set_HighAccel 0x15
#define Set_Pos_OnRange 0x16
#define Set_GearNumber 0x17
#define Read_MainGain 0x18
#define Read_SpeedGain 0x19
#define Read_IntGain 0x1a
#define Read_TrqCons 0x1b
#define Read_HighSpeed 0x1c
#define Read_HighAccel 0x1d
#define Read_Pos_OnRange 0x1e
#define Read_GearNumber 0x1f

/* Motor function codes */
#define Is_MainGain 0x10
#define Is_SpeedGain 0x11
#define Is_IntGain 0x12
#define Is_TrqCons 0x13
#define Is_HighSpeed 0x14
#define Is_HighAccel 0x15
#define Is_Drive_ID 0x16
#define Is_PosOn_Range 0x17
#define Is_GearNumber 0x18
#define Is_Status 0x19
#define Is_Config 0x1a
#define Is_AbsPos32 0x1b
#define Is_TrqCurrent 0x1e

#endif

