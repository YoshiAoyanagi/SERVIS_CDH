#ifndef PORT_CONFIG_H
#define PORT_CONFIG_H

#define H  1
#define L  0

//PCU_GPIO
#define PORT_PCU_SELECT				98
#define PORT_PCU_RESET1				99
#define PORT_PCU_RESET2				100
#define PORT_PCU_SELECT_BAT1		33
#define PORT_PCU_SELECT_BAT2		32

//Power
#define PORT_HEATER_PWR				76
#define	PORT_MSN_UNREG_PWR			77
#define PORT_RW_UNREG_PWR			78
#define	PORT_IMU_PWR				44

#define PORT_UNREG_ON				89
#define PORT_UNREG_OFF				90

//OPTIMAL-1
#define PORT_AQU_5V_PWR				38
#define PORT_OPT_5V_PWR				34

//AnalogPort for Battery
#define PORT_AN_BAT1_MON			3
#define PORT_AN_BAT2_MON			4
#define PORT_AN_SOC1_over90			5
#define PORT_AN_SOC2_over90			6

//Serial_Mux
#define PORT_SERIAL_MUX_A0			41
#define PORT_SERIAL_MUX_A1			42
#define PORT_SERIAL_MUX_EN			43

//Temperature sensor
#define	PORT_AN_EN					96
#define PORT_TEMP_RTD				2
#define PORT_TEMP_AN0				92
#define PORT_TEMP_AN1				93
#define PORT_TEMP_AN2				94
#define PORT_TEMP_AN3				95


//For SPI
#define SERIAL_CS					69
#define DR_CS						5

#define FRAM1_CS					69
#define FRAM2_CS					71
#define FLASH1_CS					73
#define FLASH2_CS					74

//ADCS
#define	ADCS_I2C_ENABLE				68

#define	SEND_ON	 1
#define SEND_OFF 0

//ADCS_I2C I/O-Expandar
#define		MTQ_IOE_PORT_PWR		0
#define		MTQ_IOE_PORT_PY			1
#define		MTQ_IOE_PORT_MY			2

#define		MTQ_IOE_PORT_PX			3
#define		MTQ_IOE_PORT_MX			4

#define		MTQ_IOE_PORT_PZ			5
#define		MTQ_IOE_PORT_MZ			6
#define		MTQ_IOE_PORT_FAULT		7


#define		STRX_IO_TX_CLK			3//4
#define		STRX_IO_TX_DATA			4//5
#define		STRX_IO_REC				5//6



#define PORT_STRX_PWR		35

#endif
