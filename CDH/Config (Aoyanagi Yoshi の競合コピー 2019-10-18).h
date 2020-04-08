#ifndef CDH_config_H
#define CDH_config_H

//Hardware

#define CPU_TICKS_CORRECT		1.041567829

#define CPU_TICKS_PER_SECOND 480 * CPU_TICKS_CORRECT

#define		UART_DEBUG
//#define		MSG_DEBUG

//pheripheral
#define BITRATE_DEBUG 115200  //bps
#define BITRATE_STRX  19200   //bps
#define BITRATE_DATA  57600  //bps
#define BITRATE_ADCS  115200  //bps
#define BITRATE_MSN	  115200  //bps
#define BITRATE_GNSSR 9600	  //bps

//AQT-D EM(FM)
#define DEBUG_Serial	Serial
#define Debug_Serial	Serial

#define DATA_Serial		Serial5
#define STRX_Serial		Serial2
#define ADCS_Serial		Serial3 
#define MSN_Serial		Serial4	
#define GNSSR_Serial	Serial1	

//TC1R�ł̃f�o�b�O���̂ݗL��
//#define DEBUG_Serial	Serial
//#define DATA_Serial		Serial1
//#define STRX_Serial		Serial2
//#define ADCS_Serial		Serial3 
//#define MSN_Serial		Serial4	
//#define GNSSR_Serial	Serial5

#endif