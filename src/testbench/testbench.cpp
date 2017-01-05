/***
* This example expects the serial port has a loopback on it.
*
* Alternatively, you could use an Arduino:
*
* <pre>
*  void setup() {
*    Serial.begin(<insert your baudrate here>);
*  }
*
*  void loop() {
*    if (Serial.available()) {
*      Serial.write(Serial.read());
*    }
*  }
* </pre>
*/

#define Go_Absolute_Pos 0x01
#define Is_AbsPos32 0x1b
#define General_Read 0x0e
#define Is_TrqCurrent 0x1e
#define Read_MainGain 0x18
#define Is_MainGain 0x10

#include <string>
#include <iostream>
#include <cstdio>

// OS Specific sleep
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#endif

#include "serial/serial.h"
#include "dmm/dmm.h"

using std::string;
using std::exception;
using std::cout;
using std::cerr;
using std::endl;
using std::vector;


uint8_t InputBuffer[256]; //Input buffer from RS232,
uint8_t OutputBuffer[256]; //Output buffer to RS232,
unsigned char InBfTopPointer, InBfBtmPointer;//input buffer pointers
unsigned char OutBfTopPointer, OutBfBtmPointer;//output buffer pointers
unsigned char Read_Package_Buffer[8], Read_Num, Read_Package_Length, Global_Func;
unsigned char MotorPosition32Ready_Flag, MotorTorqueCurrentReady_Flag, MainGainRead_Flag;
long Motor_Pos32;
int MotorTorqueCurrent, MainGain_Read;

void DlgRun::ReadPackage(serial::Serial& cSerial)
{
	unsigned char c, cif;

	while (cSerial.available()) // There is data in the customer hardware RS232 receiving Buffer
	{
		InputBuffer[InBfTopPointer] = cSerial.read(1).at(0); //Load InputBuffer with received packets
		InBfTopPointer++;
	}
	while (InBfBtmPointer != InBfTopPointer)
	{
		c = InputBuffer[InBfBtmPointer];
		InBfBtmPointer++;
		cif = c & 0x80;
		if (cif == 0)
		{
			Read_Num = 0;
			Read_Package_Length = 0;
		}
		if (cif == 0 || Read_Num>0)
		{
			Read_Package_Buffer[Read_Num] = c;
			Read_Num++;
			if (Read_Num == 2)
			{
				cif = c >> 5;
				cif = cif & 0x03;
				Read_Package_Length = 4 + cif;
				c = 0;
			}
			if (Read_Num == Read_Package_Length)
			{
				DlgRun::Get_Function();
				Read_Num = 0;
				Read_Package_Length = 0;
			}
		}
	}
}

void DlgRun::Get_Function(void)
{
	char ID, ReceivedFunction_Code, CRC_Check;
	ID = Read_Package_Buffer[0] & 0x7f;
	ReceivedFunction_Code = Read_Package_Buffer[1] & 0x1f;
	CRC_Check = 0;
	for (int i = 0; i<Read_Package_Length - 1; i++)
	{
		CRC_Check += Read_Package_Buffer[i];
	}
	CRC_Check ^= Read_Package_Buffer[Read_Package_Length - 1];
	CRC_Check &= 0x7f;
	if (CRC_Check != 0) {
		//MessageBox(“There is CRC error!”) - Customer code to indicate CRC error
	}
	else
	{
		switch (ReceivedFunction_Code) {
		case Is_AbsPos32:
			Motor_Pos32 = Cal_SignValue(Read_Package_Buffer);
			MotorPosition32Ready_Flag = 0x00;
			break;
		case Is_TrqCurrent:
			MotorTorqueCurrent = Cal_SignValue(Read_Package_Buffer);
			MotorTorqueCurrentReady_Flag = 0x00;
			break;
		case Is_MainGain:
			MainGain_Read = Cal_SignValue(Read_Package_Buffer);
			MainGainRead_Flag = 0x00;
			break;
		default:;
		}
	}
}
/*Get data with sign - long*/
long DlgRun::Cal_SignValue(unsigned char One_Package[8])
{
	char Package_Length, OneChar, i;
	long Lcmd;
	OneChar = One_Package[1];
	OneChar = OneChar >> 5;
	OneChar = OneChar & 0x03;
	Package_Length = 4 + OneChar;
	OneChar = One_Package[2]; /*First byte 0x7f, bit 6 reprents sign */
	OneChar = OneChar << 1;
	Lcmd = (long)OneChar; /* Sign extended to 32bits */
	Lcmd = Lcmd >> 1;
	for (i = 3; i<Package_Length - 1; i++)
	{
		OneChar = One_Package[i];
		OneChar &= 0x7f;
		Lcmd = Lcmd << 7;
		Lcmd += OneChar;
	}
	return(Lcmd); /* Lcmd : -2^27 ~ 2^27 - 1 */
}


//***************** Every Robot Instruction ******************
// Send a package with a function by Global_Func
// Displacement: -2^27 ~ 2^27 - 1
// Note: in the description of RS232 communication protocol above (Section 7), the last byte of packet is 
// always B0, but in the code of below, the first byte is always B0.
void DlgRun::Send_Package(char ID, long Displacement, serial::Serial& cSerial)
{
	unsigned char B[8], Package_Length, Function_Code;
	long TempLong;
	B[1] = B[2] = B[3] = B[4] = B[5] = (unsigned char)0x80;		// 10000000
	B[0] = ID & 0x7f;		// 0XXXXXXX
	Function_Code = Global_Func & 0x1f;
	TempLong = Displacement & 0x0fffffff; //Max 28bits
	B[5] += (unsigned char)TempLong & 0x0000007f;
	TempLong = TempLong >> 7;
	B[4] += (unsigned char)TempLong & 0x0000007f;
	TempLong = TempLong >> 7;
	B[3] += (unsigned char)TempLong & 0x0000007f;
	TempLong = TempLong >> 7;
	B[2] += (unsigned char)TempLong & 0x0000007f;
	Package_Length = 7;
	TempLong = Displacement;
	TempLong = TempLong >> 20;
	if ((TempLong == 0x00000000) || (TempLong == 0xffffffff))
	{//Three byte data
		B[2] = B[3];
		B[3] = B[4];
		B[4] = B[5];
		Package_Length = 6;
	}
	TempLong = Displacement;
	TempLong = TempLong >> 13;
	if ((TempLong == 0x00000000) || (TempLong == 0xffffffff))
	{//Two byte data
		B[2] = B[3];
		B[3] = B[4];
		Package_Length = 5;
	}
	TempLong = Displacement;
	TempLong = TempLong >> 6;
	if ((TempLong == 0x00000000) || (TempLong == 0xffffffff))
	{//One byte data
		B[2] = B[3];
		Package_Length = 4;
	}
	B[1] += (Package_Length - 4) * 32 + Function_Code;
	Make_CRC_Send(Package_Length, B, cSerial);
}


void DlgRun::Make_CRC_Send(unsigned char Plength, unsigned char B[8], serial::Serial& cSerial)
{
	unsigned char Error_Check = 0;
	for (int i = 0; i<Plength - 1; i++)
	{
		OutputBuffer[OutBfTopPointer] = B[i];
		OutBfTopPointer++;
		Error_Check += B[i];
	}
	Error_Check = Error_Check | 0x80;
	OutputBuffer[OutBfTopPointer] = Error_Check;
	OutBfTopPointer++;

	while (OutBfBtmPointer != OutBfTopPointer)
	{
		cSerial.write(OutputBuffer + OutBfBtmPointer, 1);
		OutBfBtmPointer++; // Change to next byte in OutputBuffer to send
	}
}

void DlgRun::ReadMotorTorqueCurrent(serial::Serial& cSerial)
{/*Below are the codes for reading the motor torque current */
 //Read motor torque current
	char ID = 0; //Suppose read 0 axis motor
	Global_Func = General_Read;
	Send_Package(ID, Is_TrqCurrent, cSerial);

	//Function code is General_Read, but one byte data is : Is_TrqCurrent
	//Then the drive will return a packet, Function code is Is_TrqCurrent
	//and the data is 16bits Motor torque current.
	MotorTorqueCurrentReady_Flag = 0xff;
	while (MotorTorqueCurrentReady_Flag != 0x00)
		ReadPackage(cSerial);
	//MotorTorqueCurrentReady_Flag is cleared inside ReadPackage() or inside
	//Get_Function() exactly after the MotorTorqueCurrent is updated.
}
void DlgRun::ReadMotorPosition32(serial::Serial& cSerial)
{/*Below are the codes for reading the motor shaft 32bits absolute position */
 //Read motor 32bits position
	char ID = 0; //Suppose read 0 axis motor
	Global_Func = General_Read;
	Send_Package(ID, Is_AbsPos32, cSerial);
	// Function code is General_Read, but one byte data is : Is_AbsPos32
	// Then the drive will return a packet, Function code is Is_AbsPos32
	// and the data is 28bits motor position32.
	MotorPosition32Ready_Flag = 0xff;
	while (MotorPosition32Ready_Flag != 0x00)
		ReadPackage(cSerial);
	// MotorPosition32Ready_Flag is cleared inside ReadPackage() or inside
	// Get_Function() exactly after the Motor_Pos32 is updated.
}
void DlgRun::MoveMotorToAbsolutePosition32(char MotorID, long Pos32, serial::Serial& cSerial)
{
	char Axis_Num = MotorID;
	Global_Func = (char)Go_Absolute_Pos;
	Send_Package(Axis_Num, Pos32, cSerial);
}
void DlgRun::ReadMainGain(char MotorID, serial::Serial& cSerial)
{
	char Axis_Num = MotorID;
	Global_Func = (char)Read_MainGain;
	Send_Package(Axis_Num, Is_MainGain, cSerial);
	MainGainRead_Flag = 0xff;
	while (MainGainRead_Flag != 0x00)
	{
		ReadPackage(cSerial);
	}
}

void my_sleep(unsigned long milliseconds) {
#ifdef _WIN32
	Sleep(milliseconds); // 100 ms
#else
	usleep(milliseconds * 1000); // 100 ms
#endif
}

void enumerate_ports()
{
	vector<serial::PortInfo> devices_found = serial::list_ports();

	vector<serial::PortInfo>::iterator iter = devices_found.begin();

	while (iter != devices_found.end())
	{
		serial::PortInfo device = *iter++;

		printf("(%s, %s, %s)\n", device.port.c_str(), device.description.c_str(),
			device.hardware_id.c_str());
	}
}

void print_usage()
{
	cerr << "Usage: test_serial {-e|<serial port address>} ";
	cerr << "<baudrate> [test string]" << endl;
}

int run(int argc, char **argv)
{
	if (argc < 2) {
		print_usage();
		return 0;
	}

	// Argument 1 is the serial port or enumerate flag
	string port(argv[1]);

	if (port == "-e") {
		enumerate_ports();
		return 0;
	}
	else if (argc < 2) {
		print_usage();
		return 1;
	}

	// port, baudrate, timeout in milliseconds
	serial::Serial cSerial(port, 38400, serial::Timeout::simpleTimeout(1000));

	cout << "Is the serial port open?";
	if (cSerial.isOpen())
		cout << " Yes." << endl;
	else
		cout << " No." << endl;


	while(true){

		DMM::ReadMotorPosition32(cSerial); //Motor absolute position stored in Motor_Pos32 variable
		cout << Motor_Pos32 << endl;

	}
	
										  

	return 0;
}

int main(int argc, char **argv) {
	try {
		return run(argc, argv);
	}
	catch (exception &e) {
		cerr << "Unhandled Exception: " << e.what() << endl;
	}
}



