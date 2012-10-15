/*
 * RS485.h
 *
 * Created: 8/21/2012 11:12:02 PM
 *  Author: Jonas
 */ 


#ifndef RS485_H_
#define RS485_H_

	#define F_CPU 16000000UL // needed for delay.h

	#include <avr/io.h>
	#include <util/delay.h>
	
	#define ENABLE_BIT_DEFINITIONS

	// Hardware Constants

	#define RS485_TXD					PORTD &= ~_BV(BIT_RS485_DIRECTION1), PORTD |= _BV(BIT_RS485_DIRECTION0)  //PORT_485_DIRECTION = 1
	#define RS485_RXD					PORTD &= ~_BV(BIT_RS485_DIRECTION0), PORTD |= _BV(BIT_RS485_DIRECTION1)  //PORT_485_DIRECTION = 0

	#define RXD0_DATA					(UDR0)
	#define RS485_TIMEOUT				50L // multiply by 10ms to get the time the read function will wait for the next byte

	#define SET_TxD0_FINISH				sbi(UCSR0A,6)
	#define RESET_TXD0_FINISH			cbi(UCSR0A,6)
	#define CHECK_TXD0_FINISH			bit_is_set(UCSR0A,6)

	#define CLEAR_BUFFER				RS485RxBufferReadPointer = RS485RxBufferWritePointer
	#define DEFAULT_RETURN_PACKET_SIZE	6
	#define DYNAMIXEL_BROADCASTING_ID	0xfe

	// --- Global Variables ---

	volatile byte RS485RxInterruptBuffer[256];
	volatile byte RS485Parameter[128];
	volatile byte RS485RxBufferReadPointer;
	volatile byte RS485RxBuffer[128];
	volatile byte RS485TxBuffer[128];
	volatile byte RS485RxBufferWritePointer;	

	#include <inttypes.h>
	typedef unsigned char byte;
	typedef unsigned int word;

	#define TXD0_READY			bit_is_set(UCSR0A,5)
	#define TXD0_DATA			(UDR0)

	void RS485Initialize(byte bBaudrate, volatile byte *RS485RxBufferReadPointer, volatile byte *RS485RxBufferWritePointer);
	void TxRS485(byte bTxdData);
	
	byte RxRS485_Packet(byte bRxPacketLength);
	byte TxRS485_Packet(byte bID, byte bInstruction, byte bParameterLength);

#endif /* RS485_H_ */