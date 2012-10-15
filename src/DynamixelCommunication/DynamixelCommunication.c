/*
 * CubeBrain.c
 *
 * Created: 4/2/2012 3:04:11 PM
 *  Author: Jonas
 */ 

#define ENABLE_BIT_DEFINITIONS
#define F_CPU				16000000UL
#define RS485_BAUD_RATE		0x01		// 1 MHz
#define DYNAMIXEL_ADDRESS	1			// we assume the Dynamixel has address 0x01

// define the output bits where we switch between read and write mode
#define BIT_RS485_DIRECTION0  PD4
#define BIT_RS485_DIRECTION1  PD5

#include <avr/io.h>
#include <inttypes.h>
#include <stdlib.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdio.h>
#include <util/delay.h>

// the following comes from Arduino's Wire library twi.h
#ifndef cbi
#define cbi(sfr, bit) (_SFR_BYTE(sfr) &= ~_BV(bit))
#endif

#ifndef sbi
#define sbi(sfr, bit) (_SFR_BYTE(sfr) |= _BV(bit))
#endif

typedef unsigned char byte;
typedef unsigned int word;

#include "RS485.c"
#include "Dynamixel.c"

/*
The following are a few helper functions for debugging and demo. You have to
add your own code for how characters are output, for example over RS232.
*/

/*
TxRS232() send data to USART 1.
*/
void TxRS232(byte bTxdData)
{
	// write your own code here for sending an individual character to some
	// output such as RS232
}

/*
	Converts a one byte numeric value to a string representation in hex format.
*/
void WriteOneByteHexValToOutput(byte bSentData)
{
	byte bTmp;

	bTmp =((byte)(bSentData>>4)&0x0f) + (byte)'0';
	if(bTmp > '9') bTmp += 7;
	TxRS232(bTmp);
	bTmp =(byte)(bSentData & 0x0f) + (byte)'0';
	if(bTmp > '9') bTmp += 7;
	TxRS232(bTmp);
}

/*
	Converts a two byte numeric value to a string representation in hex format.
*/
void WriteTwoByteHexValToOutput(uint16_t bSentData)
{
	// split the number into two bytes
	byte lsByte = bSentData & 0xff;
	byte msByte = (bSentData >> 8);
	
	byte bTmp;
	
	// send everything out
	bTmp =((byte)(msByte>>4)&0x0f) + (byte)'0';
	if(bTmp > '9') bTmp += 7;
	TxRS232(bTmp);
	bTmp =(byte)(msByte & 0x0f) + (byte)'0';
	if(bTmp > '9') bTmp += 7;
	TxRS232(bTmp);
	
	bTmp =((byte)(lsByte>>4)&0x0f) + (byte)'0';
	if(bTmp > '9') bTmp += 7;
	TxRS232(bTmp);
	bTmp =(byte)(lsByte & 0x0f) + (byte)'0';
	if(bTmp > '9') bTmp += 7;
	TxRS232(bTmp);
}

void WriteStringToOutput(char *bData)
{
	// move string into output buffer one by one
	while(*bData)
	{
		//do something for each character
	}
}



int main(void)
{	
	// set up baud rates and similar stuff
	RS485Initialize( RS485_BAUD_RATE, &RS485RxBufferReadPointer, &RS485RxBufferWritePointer );
	
	// set RS485 and inter-cube communication to Listen State.
	RS485_RXD; 
	
	//Enable Interrupt -- Compiler Function
	sei();
	
	// read all control table values from Dynamixel
	WriteStringToOutput("Dynamixel Model Number: 0x");			WriteTwoByteHexValToOutput( DynamixelGetModelNumber (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Firmware Version: 0x");		WriteOneByteHexValToOutput( DynamixelGetFirmwareVersion (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Motor ID: 0x");				WriteOneByteHexValToOutput( DynamixelGetMotorID (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Baud Rate: 0x");				WriteOneByteHexValToOutput( DynamixelGetBaudRate (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Return Delay Time: 0x");		WriteOneByteHexValToOutput( DynamixelGetReturnDelayTime (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel CW Angle Limit: 0x");		WriteTwoByteHexValToOutput( DynamixelGetCWAngleLimit (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel CCW Angle Limit: 0x");		WriteTwoByteHexValToOutput( DynamixelGetCCWAngleLimit (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Highest Limit Temp: 0x");	WriteOneByteHexValToOutput( DynamixelGetHighestLimitTemperature (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Lowest Limit Voltage: 0x");	WriteOneByteHexValToOutput( DynamixelGetLowestLimitVoltage (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Highest Limit Voltage: 0x");	WriteOneByteHexValToOutput( DynamixelGetHighestLimitVoltage (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Max Torque: 0x");			WriteTwoByteHexValToOutput( DynamixelGetMaxTorque (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Status Return Level: 0x");	WriteOneByteHexValToOutput( DynamixelGetStatusReturnLevel (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Alarm LED: 0x");				WriteOneByteHexValToOutput( DynamixelGetAlarmLED (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Alarm Shutdown: 0x");		WriteOneByteHexValToOutput( DynamixelGetAlarmShutdown (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
		
	WriteStringToOutput("Dynamixel Torque Enable: 0x");			WriteOneByteHexValToOutput( DynamixelGetTorqueEnable (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel LED: 0x");					WriteOneByteHexValToOutput( DynamixelGetLED (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel CW Compliance Margin: 0x");	WriteOneByteHexValToOutput( DynamixelGetCWComplianceMargin (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel CW Compliance Slope: 0x");	WriteOneByteHexValToOutput( DynamixelGetCWComplianceSlope (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel CCW Compliance Margin: 0x");	WriteOneByteHexValToOutput( DynamixelGetCCWComplianceMargin (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel CCW Compliance Slope: 0x");	WriteOneByteHexValToOutput( DynamixelGetCCWComplianceSlope (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Goal Position: 0x");			WriteTwoByteHexValToOutput( DynamixelGetGoalPosition (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Goal Speed: 0x");			WriteTwoByteHexValToOutput( DynamixelGetGoalSpeed (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n"); // this is a limit setting, not the current speed of the motor
	WriteStringToOutput("Dynamixel Torque Limit: 0x");			WriteTwoByteHexValToOutput( DynamixelGetTorqueLimit (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Current Position: 0x");		WriteTwoByteHexValToOutput( DynamixelGetMotorPosition (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Current Speed: 0x");			WriteTwoByteHexValToOutput( DynamixelGetSpeed (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Current Load: 0x");			WriteTwoByteHexValToOutput( DynamixelGetLoad (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Current Voltage: 0x");		WriteOneByteHexValToOutput( DynamixelGetVoltage (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Current Temperature: 0x");	WriteOneByteHexValToOutput( DynamixelGetTemperature (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Currently Moving: 0x");		WriteOneByteHexValToOutput( DynamixelGetMoving (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");
	WriteStringToOutput("Dynamixel Punch: 0x");					WriteTwoByteHexValToOutput( DynamixelGetPunch (DYNAMIXEL_ADDRESS) ); WriteStringToOutput("\r\n");	
	
	_delay_ms(1000);
	
	// set Dynamixel Control Table Values
	DynamixelSetMotorID( DYNAMIXEL_ADDRESS, 0x01 );
	DynamixelSetBaudRate( DYNAMIXEL_ADDRESS, 0x01 );
	DynamixelSetReturnDelayTime( DYNAMIXEL_ADDRESS, 0xFA );
	DynamixelSetCWAngleLimit( DYNAMIXEL_ADDRESS, 0x0000 );
	DynamixelSetCCWAngleLimit( DYNAMIXEL_ADDRESS, 0x03FF );
	DynamixelSetHighestLimitTemperature( DYNAMIXEL_ADDRESS, 0x55 );
	DynamixelSetLowestLimitVoltage( DYNAMIXEL_ADDRESS, 0x3C );
	DynamixelSetHighestLimitVoltage( DYNAMIXEL_ADDRESS, 0xBE );
	DynamixelSetMaxTorque( DYNAMIXEL_ADDRESS, 0x03FF );
	DynamixelSetStatusReturnLevel( DYNAMIXEL_ADDRESS, 0x02 );
	DynamixelSetAlarmLED( DYNAMIXEL_ADDRESS, 0x04 );
	DynamixelSetAlarmShutdown( DYNAMIXEL_ADDRESS, 0x04 );
	DynamixelSetTorqueEnable( DYNAMIXEL_ADDRESS, 0x01 );
	DynamixelSetLED( DYNAMIXEL_ADDRESS, 0x00 );
	DynamixelSetCWComplianceMargin( DYNAMIXEL_ADDRESS, 0x00 );
	DynamixelSetCCWComplianceMargin( DYNAMIXEL_ADDRESS, 0x00 );
	DynamixelSetCWComplianceSlope( DYNAMIXEL_ADDRESS, 0x20 );
	DynamixelSetCCWComplianceSlope( DYNAMIXEL_ADDRESS, 0x20 );
	DynamixelSetGoalPosition( DYNAMIXEL_ADDRESS, 0x0100 );
	DynamixelSetGoalSpeed( DYNAMIXEL_ADDRESS, 0x0200 );
	DynamixelSetTorqueLimit( DYNAMIXEL_ADDRESS, 0x03FF );
	DynamixelSetPunch( DYNAMIXEL_ADDRESS, 0x040 );
	
	_delay_ms(1000);
	
	// blink LED
	for (int i=0; i<5; i++)
	{
		DynamixelSetLED( DYNAMIXEL_ADDRESS, 0x01 );	
		_delay_ms(200);
		DynamixelSetLED( DYNAMIXEL_ADDRESS, 0x00 );	
		_delay_ms(500);
	}
	
	// continuous rotation
	DynamixelEnableContinuousRotation(DYNAMIXEL_ADDRESS);
	DynamixelSetGoalSpeed(DYNAMIXEL_ADDRESS, 0x0200);
	
	_delay_ms(1000);
	
	// reverse direction
	DynamixelSetGoalSpeed(DYNAMIXEL_ADDRESS, 0x0200 ^ 0x0400);
	
	_delay_ms(1000);
	
	// leave continuous rotation mode
	DynamixelDisableContinuousRotation(DYNAMIXEL_ADDRESS);
	
	// move to target position
	DynamixelMoveTo(DYNAMIXEL_ADDRESS, 0x0300, 0x0200 ); 
	
	
	// end of program
	while(1);
}