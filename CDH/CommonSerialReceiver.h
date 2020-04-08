// CommonSerialReceiver.h

#ifndef _COMMONSERIALRECEIVER_h
#define _COMMONSERIALRECEIVER_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "CommonTlmCmdPacket.h"
#include "FRAM_manager.h"
#include "Utility.h"

typedef enum
{
	REC_STATUS_0,
	REC_STATUS_STX_1,
	REC_STATUS_STX_2,
	REC_STATUS_LENGTH,
	REC_STATUS_ETX_1,
	REC_STATUS_ETX_2
};

typedef enum
{
	PORT_DBG,
	PORT_DATA,
	PORT_STRX,
	PORT_ADCS,
	PORT_SF,
	PORT_SUBCAM,
	PORT_AQU,
	PORT_MSN
} FOWARD_PORT;

typedef enum
{
	MUX_NONE	= 0x00,
	MUX_RPI		= 0x01,
	MUX_MSN1	= 0x03,
	MUX_MSN2	= 0x02,
	MUX_CAM		= 0x04
} SERIAL_MUX;

extern SERIAL_MUX	serial_mux_enable_port;
ack Serial_DataReceive(FOWARD_PORT port, unsigned char *data, int *len);
void Serial_MUX_init(void);
ack Serial_MUX_change(SERIAL_MUX port);

void SerialBitStream(unsigned char *data, unsigned int len);
void TestSerialBitStream(void);

#endif