// 
// 
// 

#include "CommonSerialReceiver.h"

SERIAL_MUX	serial_mux_enable_port;

//Serial受信処理//////////////////////////
ack Serial_DataReceive(FOWARD_PORT port, unsigned char *data, int *len) {

	int i_ret = 0;
	int receive_size = 0;
	int packet_length = 0;
	int timeout = 0;
	int rec_sts = REC_STATUS_0;
	int header_offset = 0;

	while (rec_sts != REC_STATUS_ETX_2)
	{
		switch (port)
		{
			case PORT_DATA:		i_ret = DATA_Serial.available(); break;
			case PORT_STRX:		i_ret = STRX_Serial.available(); break;
			case PORT_ADCS:		i_ret = ADCS_Serial.available(); break;
			case PORT_SF:		i_ret = MSN_Serial.available(); break;
			case PORT_SUBCAM:	i_ret = MSN_Serial.available(); ; break;
			case PORT_AQU:		i_ret = MSN_Serial.available(); break;
			case PORT_MSN:		i_ret = MSN_Serial.available(); break;
			case PORT_DBG:		i_ret = DEBUG_Serial.available(); break;
			default:			break;
		}
		timeout++;

		if (timeout > 0xFFFF)
		{
			Serial.println("serial receive timeout !!!");
			return (ValueNull);
		}

		if (i_ret > 0)
		{
			if (i_ret > 0x0FFF)	//オーバーフロー対策
				i_ret = 0x0FFF;

			for (int i = 0; i < i_ret; i++)
			{
				switch (port)
				{
				case PORT_DATA:		data[receive_size] = DATA_Serial.read(); break;// Serial.print(data[receive_size], HEX); break;
				case PORT_STRX:		data[receive_size] = STRX_Serial.read(); break;
				case PORT_ADCS:		data[receive_size] = ADCS_Serial.read(); break;
				case PORT_SF:		data[receive_size] = MSN_Serial.read(); break;
				case PORT_SUBCAM:	data[receive_size] = MSN_Serial.read(); break;
				case PORT_AQU:		data[receive_size] = MSN_Serial.read(); break;
				case PORT_MSN:		data[receive_size] = MSN_Serial.read(); break;// Serial.print(data[receive_size], HEX); Serial.print(" "); break;
				case PORT_DBG:		data[receive_size] = DEBUG_Serial.read(); break;
				default:			break;

				}
				if (rec_sts == REC_STATUS_0)
				{
					if (data[receive_size] == CMD_STX1)
					{
						rec_sts = REC_STATUS_STX_1;
						header_offset = receive_size;
					}
				}
				else if (rec_sts == REC_STATUS_STX_1)
				{
					if (data[receive_size] == CMD_STX2)
					{
						rec_sts = REC_STATUS_STX_2;
					}
				}
				else if (rec_sts == REC_STATUS_STX_2)		//length
				{
					packet_length = data[receive_size];
					rec_sts = REC_STATUS_LENGTH;
				}
				else if (rec_sts == REC_STATUS_LENGTH)
				{
					
					if (receive_size >= (packet_length + CCP_STX_ETX_LENGTH + header_offset))
					{
						rec_sts = REC_STATUS_ETX_2;
						*len = receive_size + 1;
						
						return (Success);
					}
				}
				receive_size++;
				if (receive_size > 255)
				{
					Serial.println("serial receive overflow !!!");
					return (ValueNull);
				}
			}
		}
		if (rec_sts == REC_STATUS_0)
			break;
	}
	return (ValueNull);
}


void Serial_MUX_init(void)
{
	//Serial_Mux
	pinMode(PORT_SERIAL_MUX_A0, OUTPUT);
	pinMode(PORT_SERIAL_MUX_A1, OUTPUT);
	pinMode(PORT_SERIAL_MUX_EN, OUTPUT);
	digitalWrite(PORT_SERIAL_MUX_EN, HIGH);

	unsigned char port = MUX_MSN1;
	ack i_ret = FRAM_Read_serial_mux(&port);
	if (Serial_MUX_change((SERIAL_MUX)port) != Success)
	{
		serial_mux_enable_port = MUX_MSN1;
		Serial_MUX_change(serial_mux_enable_port);
	}
}

ack Serial_MUX_change(SERIAL_MUX port)
{
	if (port == MUX_RPI)
	{
		MSN_Serial.end();
		MSN_Serial.begin(BITRATE_MSN);
		digitalWrite(PORT_SERIAL_MUX_A0, LOW);
		digitalWrite(PORT_SERIAL_MUX_A1, LOW);
		serial_mux_enable_port = MUX_RPI;
		FRAM_Write_serial_mux((SERIAL_MUX)port);//FRAMに保存
		return Success;
	}
	else if (port == MUX_MSN1)
	{
		MSN_Serial.end();
		MSN_Serial.begin(BITRATE_MSN);
		digitalWrite(PORT_SERIAL_MUX_A0, HIGH);
		digitalWrite(PORT_SERIAL_MUX_A1, LOW);
		serial_mux_enable_port = MUX_MSN1;
		FRAM_Write_serial_mux((SERIAL_MUX)port);//FRAMに保存
		return Success;
	}
	else if (port == MUX_MSN2)
	{
		MSN_Serial.end();
		MSN_Serial.begin(BITRATE_MSN);
		digitalWrite(PORT_SERIAL_MUX_A0, LOW);
		digitalWrite(PORT_SERIAL_MUX_A1, HIGH);
		serial_mux_enable_port = MUX_MSN2;
		FRAM_Write_serial_mux((SERIAL_MUX)port);//FRAMに保存
		return Success;
	}
	else if (port == MUX_CAM)
	{
		MSN_Serial.end();
		MSN_Serial.begin(38400);
		digitalWrite(PORT_SERIAL_MUX_A0, HIGH);
		digitalWrite(PORT_SERIAL_MUX_A1, HIGH);
		serial_mux_enable_port = MUX_CAM;

		FRAM_Write_serial_mux((SERIAL_MUX)port);//FRAMに保存
		return Success;
	}
	else
		return Error;
	
}

void SerialBitStream(unsigned char *data, unsigned int len)
{
	unsigned int us = 25;	//5useconds
	unsigned int fus = 1;	//5useconds

	int i, j;
	bool bit_val;
	int bit_pos = 0;
	int data_pos = 0;

	for (j = 0; j < len; j++)
	{
		for (i = 0; i < 8; i++)
		{
			bit_val = (data[data_pos] >> (7 - bit_pos)) & 0x01;
			if (bit_val)
			{
				digitalWrite(STRX_IO_TX_DATA, HIGH); //PORTEbits.RE6 = 1;//
			}
			else
			{
				digitalWrite(STRX_IO_TX_DATA, LOW);  //PORTEbits.RE6 = 0;// 
			}
			delayMicroseconds(fus);
			PORTEbits.RE5 = 1; //digitalWrite(STRX_IO_TX_CLK, HIGH);
			delayMicroseconds(us-fus);
			PORTEbits.RE5 = 0; //digitalWrite(STRX_IO_TX_CLK, LOW);
			delayMicroseconds(us);
			bit_pos++;
		}
		bit_pos = 0;
		data_pos++;
	}
	digitalWrite(STRX_IO_TX_DATA, LOW); //PORTEbits.RE6 = 0;//
	PORTEbits.RE5 = 0;//digitalWrite(STRX_IO_TX_CLK, LOW);
}

void TestSerialBitStream(void)
{
	unsigned char data[256];
	int i;
	for (i = 0; i < 256; i++)
	{
		data[i] = 0xB5;
	}
	SerialBitStream(data, 256);

}
