// STRX_update.h

#ifndef _STRX_UPDATE_h
#define _STRX_UPDATE_h

#if defined(ARDUINO) && ARDUINO >= 100
	#include "arduino.h"
#else
	#include "WProgram.h"
#endif

#include "AppInfo.h"
#include "AppHeaders.h"
#include "STRX.h"

#define STRX_IF_BR4KBPS        0x00
#define STRX_IF_BR32KBPS       0x01
#define STRX_IF_BR64KBPS       0x02
#define STRX_IF_CMD_TX_ON      0x0A
#define STRX_IF_CMD_TX_OFF     0x0B

#define STRX_IF_STX1  0xEB
#define STRX_IF_STX2  0x90
#define STRX_IF_ETX1  0xC5
#define STRX_IF_ETX2  0x79

#define	DEFAULT_STX_TRANSMIT_ON_DURATION	30
#define	DEFAULT_STX_TRANSMIT_OFF_DURATION	30

#define	STRX_OTC_DEFAULT_TEMPERATURE		60
#define	STRX_OTC_LOW_TEMPERATURE			30
#define STRX_OTC_THR_COUNT					10

extern unsigned int stx_transmit_on_duration;
extern unsigned int stx_transmit_off_duration;
extern unsigned char stx_transmit_on;
extern unsigned int stx_transmit_count;
extern unsigned int stx_burst_duration;
extern unsigned int stx_on_remaining;
extern unsigned char strx_otc_enable;
extern unsigned char strx_otc_threshold;


AppInfo STRX_update(void);
AppInfo STX_transmit(void);

void STRX_IF_Cmd(unsigned char param);

void STRX_IF_Cmd_BR4KBPS(void);
void STRX_IF_Cmd_BR32KBPS(void); 
void STRX_IF_Cmd_BR64KBPS(void);
void STRX_IF_Cmd_TX_ON(void);
void STRX_IF_Cmd_TX_OFF(void);

void STRX_Cmd_BR4KBPS(void);
void STRX_Cmd_BR32KBPS(void);
void STRX_Cmd_BR64KBPS(void);
void STRX_Cmd_PWR(unsigned char pwr_level);
void STRX_OTC_Detect(void);;

void STRX_set_OTC_Enable(unsigned char en);
void STRX_set_OTC_Threshold(unsigned char val);
void STX_set_ON(void);
void STX_set_OFF(void);
void STX_set_transmit_duration(unsigned int on_duration, unsigned int off_duration);
void STX_set_burst_duration(unsigned int duration);

#endif

