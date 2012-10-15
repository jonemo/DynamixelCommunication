/*
 * Dynamixel.c
 *
 * Created: 5/4/2012 6:27:37 PM
 *  Author: Jonas
 */ 


#include "Dynamixel.h"


byte DynamixelInitialize( void )
{
	// check if a Dynamixel is responding on address 1
	byte bRxPacketLength;
	RS485Parameter[0] = P_PRESENT_VOLTAGE;
	RS485Parameter[1] = 1; //Read Length
	TxRS485_Packet(DYNAMIXEL_ADDRESS, INST_READ, 2);
	bRxPacketLength = RxRS485_Packet(DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1]);
	
	// if the Dynamixel did not return anything, return 0 = false.
	if (bRxPacketLength == 0) return 0;
	
	// Do some motor setup
	DynamixelSetComplianceSlopes (DYNAMIXEL_ADDRESS, 0x20); // default = 0x20
	DynamixelSetComplianceMargins (DYNAMIXEL_ADDRESS, 0); // default = 0
	DynamixelSetPunch(DYNAMIXEL_ADDRESS, 0x40); // default value seems to be 0x20 (also the minimum)
	DynamixelSetTorqueEnable (DYNAMIXEL_ADDRESS, 1);
	DynamixelDisableContinuousRotation (DYNAMIXEL_ADDRESS);
	
	// it worked, return 1 = true
	return 1;
}


uint8_t DynamixelGetOneByteValue(byte servoId, byte controlTablePosition)
{
	byte bRxPacketLength;
	RS485Parameter[0] = controlTablePosition;
	RS485Parameter[1] = 1; //Read Length
	TxRS485_Packet(servoId,INST_READ,2);
	bRxPacketLength = RxRS485_Packet(DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1]);
	 
	if( bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1] )
	{
		return RS485RxBuffer[5];
	}
	
	_delay_ms(50);
	
	return 0xff;
}


uint16_t DynamixelGetTwoByteValue (byte servoId, byte controlTablePosition)
{
	byte bRxPacketLength;
	
	RS485Parameter[0] = controlTablePosition;
	RS485Parameter[1] = 2; //Read Length
	
	TxRS485_Packet( servoId, INST_READ, 2 );
	bRxPacketLength = RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1] );
	
	if( bRxPacketLength == DEFAULT_RETURN_PACKET_SIZE + RS485Parameter[1] )
	{
		return (RS485RxBuffer[6]<<8) + RS485RxBuffer[5];
	}
	
	_delay_ms(50);
	
	return 0xffff;
}


void DynamixelSetOneByteValue(byte servoID, byte controlTablePosition, uint8_t value)
{
	RS485Parameter[0] = controlTablePosition;
	RS485Parameter[1] = value;
	TxRS485_Packet(servoID,INST_WRITE,2);

	// wait for the reply
	RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE );

	_delay_ms(50);

	return;
}


void DynamixelSetTwoByteValue(byte servoID, byte controlTablePosition, uint16_t value)
{
	byte th, tl;

	th = value >> 8;
	tl = value & 0xff;

	RS485Parameter[0] = controlTablePosition;
	RS485Parameter[1] = tl;
	RS485Parameter[2] = th;
	TxRS485_Packet(servoID,INST_WRITE,3);
	
	// wait for the reply
	RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE );

	_delay_ms(50);
	
	return;
}


/************************************************************************/
/* Getter Functions                                                     */
/************************************************************************/


uint16_t DynamixelGetModelNumber (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_MODEL_NUMBER_L);
}


uint8_t DynamixelGetFirmwareVersion (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_VERSION);
}


uint8_t DynamixelGetMotorID (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_ID);
}


uint8_t DynamixelGetBaudRate (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_BAUD_RATE);
}


uint8_t DynamixelGetReturnDelayTime (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_RETURN_DELAY_TIME);
}


uint16_t DynamixelGetCWAngleLimit (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_CW_ANGLE_LIMIT_L);
}


uint16_t DynamixelGetCCWAngleLimit (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_CCW_ANGLE_LIMIT_L);
}


uint8_t DynamixelGetHighestLimitTemperature (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_LIMIT_TEMPERATURE);
}


uint8_t DynamixelGetLowestLimitVoltage (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_DOWN_LIMIT_VOLTAGE);
}


uint8_t DynamixelGetHighestLimitVoltage (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_UP_LIMIT_VOLTAGE);
}


uint16_t DynamixelGetMaxTorque (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_MAX_TORQUE_L);
}


uint8_t DynamixelGetStatusReturnLevel (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_RETURN_LEVEL);
}


uint8_t DynamixelGetAlarmLED (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_ALARM_LED);
}


uint8_t DynamixelGetAlarmShutdown (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_ALARM_SHUTDOWN);
}


uint8_t DynamixelGetTorqueEnable (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_TORQUE_ENABLE);
}


uint8_t DynamixelGetLED (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_LED);
}


uint8_t DynamixelGetCWComplianceMargin (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_CW_COMPLIANCE_MARGIN);
}


uint8_t DynamixelGetCCWComplianceMargin (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_CCW_COMPLIANCE_MARGIN);
}


uint8_t DynamixelGetCWComplianceSlope (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_CW_COMPLIANCE_SLOPE);
}


uint8_t DynamixelGetCCWComplianceSlope (byte servoID)
{
	return DynamixelGetOneByteValue (servoID, P_CCW_COMPLIANCE_SLOPE);
}


uint16_t DynamixelGetGoalPosition (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_GOAL_POSITION_L);
}


uint16_t DynamixelGetGoalSpeed (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_GOAL_SPEED_L);
}


uint16_t DynamixelGetTorqueLimit (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_TORQUE_LIMIT_L);
}


uint16_t DynamixelGetMotorPosition (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_PRESENT_POSITION_L);
}


uint16_t DynamixelGetSpeed (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_PRESENT_SPEED_L);
}


uint16_t DynamixelGetLoad (byte servoID) // this is the current torque
{
	return DynamixelGetTwoByteValue (servoID, P_PRESENT_LOAD_L);
}


uint8_t DynamixelGetVoltage (byte servoID)
{
	return DynamixelGetOneByteValue(servoID, P_PRESENT_VOLTAGE);
}


uint8_t DynamixelGetTemperature (byte servoID)
{
	return DynamixelGetOneByteValue(servoID, P_PRESENT_TEMPERATURE);
}


uint8_t DynamixelGetMoving (byte servoID)
{
	return DynamixelGetOneByteValue(servoID, P_MOVING);
}


uint16_t DynamixelGetPunch (byte servoID)
{
	return DynamixelGetTwoByteValue (servoID, P_PUNCH_L);
}


/************************************************************************/
/* Setter Functions                                                     */
/************************************************************************/

void DynamixelSetMotorID (byte servoID, uint8_t theID)
{
	if (theID > 0xfd) return;
	DynamixelSetOneByteValue(servoID, P_ID, theID);
}


/*
Baud Rate. Determines the communication speed. The computation is done by the following formula.
Speed (BPS) = 2000000 / (Address4 + 1)
Note A maximum Baud Rate error of 3% is within the tolerance of UART communication.
Caution The initial value of Baudrate is set to 1(1000000bps)
*/
void DynamixelSetBaudRate (byte servoID, uint8_t baudrate)
{
	if (baudrate > 0xfe) return;
	DynamixelSetOneByteValue(servoID, P_BAUD_RATE, baudrate);
}


/*
Return Delay Time. The time it takes for the Status Packet to return after the Instruction Packet is sent. 
The delay time is given by 2uSec * Address5 value.
*/
void DynamixelSetReturnDelayTime (byte servoID, uint8_t rdt)
{
	if (rdt > 0xfe) return;
	DynamixelSetOneByteValue(servoID, P_RETURN_DELAY_TIME, rdt);
}


void DynamixelSetCWAngleLimit (byte servoID, uint16_t cwAngleLimit)
{
	if (cwAngleLimit > 0x3ff) return;
	DynamixelSetTwoByteValue(servoID, P_CW_ANGLE_LIMIT_L, cwAngleLimit);
}


void DynamixelSetCCWAngleLimit (byte servoID, uint16_t ccwAngleLimit)
{
	if (ccwAngleLimit > 0x3ff) return;
	DynamixelSetTwoByteValue(servoID, P_CCW_ANGLE_LIMIT_L, ccwAngleLimit);
}


void DynamixelSetHighestLimitTemperature (byte servoID, uint8_t hlt)
{
	if (hlt > 0x96) return;
	DynamixelSetOneByteValue(servoID, P_LIMIT_TEMPERATURE, hlt);
}


void DynamixelSetLowestLimitVoltage (byte servoID, uint8_t llv)
{
	if (llv < 0x32 || llv > 0xfa) return;
	DynamixelSetOneByteValue(servoID, P_DOWN_LIMIT_VOLTAGE, llv);
}


void DynamixelSetHighestLimitVoltage (byte servoID, uint8_t hlv)
{
	if (hlv < 0x32 || hlv > 0xfa) return;
	DynamixelSetOneByteValue(servoID, P_UP_LIMIT_VOLTAGE, hlv);
}


void DynamixelSetMaxTorque (byte servoID, uint16_t maxTorque)
{
	if (maxTorque > 0x3ff) return;
	DynamixelSetTwoByteValue(servoID, P_MAX_TORQUE_L, maxTorque);
}


void DynamixelSetStatusReturnLevel (byte servoID, uint8_t srl)
{
	if (srl > 2) return;
	DynamixelSetOneByteValue(servoID, P_RETURN_LEVEL, srl);
}


void DynamixelSetAlarmLED (byte servoID, uint8_t alarmLed)
{
	if (alarmLed > 0x7f) return;
	DynamixelSetOneByteValue(servoID, P_ALARM_LED, alarmLed);
}


void DynamixelSetAlarmShutdown (byte servoID, uint8_t alarmShutdown)
{
	if (alarmShutdown > 0x7f) return;
	DynamixelSetOneByteValue(servoID, P_ALARM_SHUTDOWN, alarmShutdown);
}


/*
Torque Enable. When the power is first turned on, the Dynamixel actuator enters the Torque Free 
Run condition (zero torque). Setting the value in Address 0x18 to 1 enables the torque.
*/
void DynamixelSetTorqueEnable(byte servoID,byte onoff01)
{
	if (onoff01 != 0 && onoff01 != 1) return;
	
	DynamixelSetOneByteValue(servoID, P_TORQUE_ENABLE, onoff01);
}


void DynamixelSetLED(byte servoID,byte onOff01)
{
	if (onOff01 != 0 && onOff01 != 1) return;
	
	DynamixelSetOneByteValue(servoID, P_LED, onOff01);
}


void DynamixelSetCWComplianceMargin (byte servoID, uint8_t cwComplianceMargin)
{
	if (cwComplianceMargin > 0xfe) return;
	DynamixelSetOneByteValue(servoID, P_CW_COMPLIANCE_MARGIN, cwComplianceMargin);
}


void DynamixelSetCCWComplianceMargin (byte servoID, uint8_t ccwComplianceMargin)
{
	if (ccwComplianceMargin > 0xfe) return;
	DynamixelSetOneByteValue(servoID, P_CCW_COMPLIANCE_MARGIN, ccwComplianceMargin);
}


void DynamixelSetCWComplianceSlope (byte servoID, uint8_t cwComplianceSlope)
{
	if (cwComplianceSlope > 0xfe) return;
	DynamixelSetOneByteValue(servoID, P_CW_COMPLIANCE_SLOPE, cwComplianceSlope);
}


void DynamixelSetCCWComplianceSlope (byte servoID, uint8_t ccwComplianceSlope)
{
	if (ccwComplianceSlope > 0xfe) return;
	DynamixelSetOneByteValue(servoID, P_CCW_COMPLIANCE_SLOPE, ccwComplianceSlope);
}


/*
Goal Position Requested angular position for the Dynamixel actuator output to move to. Setting 
this value to 0x3ff moves the output shaft to the position at 300°.
*/
void DynamixelSetGoalPosition (byte servoID,uint16_t goalPosition)
{
	if (goalPosition > 0x3ff) return;
	DynamixelSetTwoByteValue(servoID, P_GOAL_POSITION_L, goalPosition);
}


/*
Moving Speed. Sets the angular velocity of the output moving to the Goal Position. Setting this 
value to its maximum value of 0x3ff moves the output with an angular velocity of 114 RPM, 
provided that there is enough power supplied (The lowest velocity is when this value is set to 1. 
When set to 0, the velocity is the largest possible for the supplied voltage, e.g. no velocity 
control is applied.)
*/
void DynamixelSetGoalSpeed (byte servoID, uint16_t movingSpeed)
{
	if (movingSpeed > 0x7ff) return; // Note: the 10th bit can be used in endless turn mode to control the direction
	DynamixelSetTwoByteValue(servoID, P_GOAL_SPEED_L, movingSpeed);
}


void DynamixelSetTorqueLimit (byte servoID,uint16_t torqueLimit)
{
	if (torqueLimit > 0x3ff) return;
	DynamixelSetTwoByteValue(servoID, P_TORQUE_LIMIT_L, torqueLimit);
}


void DynamixelSetPunch (byte servoID, uint16_t punch)
{
	if (punch > 0x3ff) return;
	DynamixelSetTwoByteValue(servoID, P_PUNCH_L, punch);
}



/************************************************************************/
/* Helper Functions                                                     */
/************************************************************************/

void DynamixelSetComplianceMargins (byte servoID, uint8_t complianceMargin)
{
	DynamixelSetCWComplianceMargin(servoID, complianceMargin);
	DynamixelSetCCWComplianceMargin(servoID, complianceMargin);
}


void DynamixelSetComplianceSlopes (byte servoID, uint8_t complianceSlope)
{
	DynamixelSetCWComplianceSlope(servoID, complianceSlope);
	DynamixelSetCCWComplianceSlope(servoID, complianceSlope);
}


void DynamixelMoveTo(byte servoID, uint16_t pos, uint16_t speed)
{

	byte ph,pl,sh,sl;

	ph = pos >> 8;
	pl = pos & 0xff;
	sh = speed >> 8;
	sl = speed & 0xff;
	
	RS485Parameter[0] = P_GOAL_POSITION_L;
	RS485Parameter[1] = pl; //Writing Data P_GOAL_POSITION_L
	RS485Parameter[2] = ph; //Writing Data P_GOAL_POSITION_H
	RS485Parameter[3] = sl; //Writing Data P_GOAL_SPEED_L
	RS485Parameter[4] = sh; //Writing Data P_GOAL_SPEED_H
	TxRS485_Packet(servoID,INST_WRITE,5);
	
	// wait for the reply
	RxRS485_Packet( DEFAULT_RETURN_PACKET_SIZE );
}


/*
If both values for the CW Angle Limit and the CCW Angle Limit are set to 0, an Endless Turn mode 
can be implemented by setting the Goal Speed. This feature can be used for implementing a continuously 
rotating wheel.
*/
void DynamixelEnableContinuousRotation (byte servoID)
{
	DynamixelSetCWAngleLimit(servoID, 0x00);
	//_delay_ms(10); // without this line the second dynamixel instruction has a read timeout
	DynamixelSetCCWAngleLimit(servoID, 0x00);
}

void DynamixelDisableContinuousRotation (byte servoID)
{
	DynamixelSetCWAngleLimit(servoID, 0x00);
	//_delay_ms(10); // without this line the second dynamixel instruction has a read timeout
	DynamixelSetCCWAngleLimit(servoID, 0x3ff);
}